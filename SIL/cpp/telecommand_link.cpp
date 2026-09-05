#include "telecommand_link.h"

#include "crc32.h"

#include <cstddef>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace {

bool setNonBlocking(socket_t sock) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(sock, FIONBIO, &mode) == 0;
#else
    const int flags = fcntl(sock, F_GETFL, 0);
    return flags >= 0 && fcntl(sock, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

bool wouldBlock() {
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

bool unpackTelecommand(const char* raw, TelecommandPacket& out) {
    std::memcpy(&out, raw, sizeof(out));
    const uint32_t expected = crc32util::compute(
        &out,
        sizeof(TelecommandPacket) - sizeof(out.crc32)
    );
    return out.header.healthCheck == kCommandHealthCheck
        && out.header.version == kPacketVersion
        && out.header.packet_type == PACKET_TELECOMMAND
        && out.crc32 == expected;
}

}  // namespace

TelecommandLink::~TelecommandLink() {
    close();
}

bool TelecommandLink::listen(std::uint16_t port) {
    close();

    listen_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_ == kInvalidSocket) {
        std::cerr << "TC socket() failed\n";
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    int reuse = 1;
    setsockopt(
        listen_,
        SOL_SOCKET,
        SO_REUSEADDR,
        reinterpret_cast<const char*>(&reuse),
        sizeof(reuse)
    );

    if (bind(listen_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "TC bind() failed on port " << port << "\n";
        close();
        return false;
    }
    if (::listen(listen_, 1) != 0) {
        std::cerr << "TC listen() failed\n";
        close();
        return false;
    }
    if (!setNonBlocking(listen_)) {
        std::cerr << "TC listen setNonBlocking failed\n";
        close();
        return false;
    }

    std::cout << "TC console listening on port " << port << "\n";
    return true;
}

void TelecommandLink::poll(std::vector<TelecommandPacket>& out) {
    if (listen_ == kInvalidSocket) {
        return;
    }
    acceptIfPending();
    drainClient(out);
}

void TelecommandLink::close() {
    dropClient();
    closeSocket(listen_);
    listen_ = kInvalidSocket;
}

void TelecommandLink::acceptIfPending() {
    sockaddr_in from{};
#ifdef _WIN32
    int fromLen = sizeof(from);
#else
    socklen_t fromLen = sizeof(from);
#endif
    const socket_t incoming = accept(
        listen_,
        reinterpret_cast<sockaddr*>(&from),
        &fromLen
    );
    if (incoming == kInvalidSocket) {
        return;
    }
    if (!setNonBlocking(incoming)) {
        closeSocket(incoming);
        return;
    }
    if (client_ != kInvalidSocket) {
        std::cout << "TC console replaced previous client\n";
        dropClient();
    }
    client_ = incoming;
    rx_.clear();
    std::cout << "TC console connected\n";
}

void TelecommandLink::drainClient(std::vector<TelecommandPacket>& out) {
    if (client_ == kInvalidSocket) {
        return;
    }

    char chunk[128];
    while (true) {
#ifdef _WIN32
        const int n = recv(client_, chunk, sizeof(chunk), 0);
#else
        const ssize_t n = recv(client_, chunk, sizeof(chunk), 0);
#endif
        if (n > 0) {
            rx_.insert(rx_.end(), chunk, chunk + n);
            continue;
        }
        if (n == 0) {
            std::cout << "TC console disconnected\n";
            dropClient();
            return;
        }
        if (wouldBlock()) {
            break;
        }
        std::cout << "TC console recv failed\n";
        dropClient();
        return;
    }

    while (rx_.size() >= kTelecommandPacketSize) {
        TelecommandPacket packet{};
        if (!unpackTelecommand(rx_.data(), packet)) {
            std::cerr << "TC packet rejected (framing/CRC)\n";
            rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(kTelecommandPacketSize));
            continue;
        }
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(kTelecommandPacketSize));
        out.push_back(packet);
    }
}

void TelecommandLink::dropClient() {
    closeSocket(client_);
    client_ = kInvalidSocket;
    rx_.clear();
}
