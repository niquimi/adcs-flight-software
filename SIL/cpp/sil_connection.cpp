#include "sil_connection.h"

#include "boot_config_receiver.h"
#include "command_sender.h"
#include "flight_software.h"
#include "sensor_packet.h"
#include "sensor_receiver.h"
#include "types.h"
#include "fault_injector.h"

#include <iostream>

bool recvExact(socket_t sock, char* buffer, std::size_t size) {
    std::size_t received = 0;
    while (received < size) {
#ifdef _WIN32
        const int chunk = recv(
            sock,
            buffer + received,
            static_cast<int>(size - received),
            0
        );
#else
        const ssize_t chunk = recv(sock, buffer + received, size - received, 0);
#endif
        if (chunk <= 0) {
            return false;
        }
        received += static_cast<std::size_t>(chunk);
    }
    return true;
}

bool sendExact(socket_t sock, const char* buffer, std::size_t size) {
    std::size_t sent = 0;
    while (sent < size) {
#ifdef _WIN32
        const int chunk = send(
            sock,
            buffer + sent,
            static_cast<int>(size - sent),
            0
        );
#else
        const ssize_t chunk = send(sock, buffer + sent, size - sent, 0);
#endif
        if (chunk <= 0) {
            return false;
        }
        sent += static_cast<std::size_t>(chunk);
    }
    return true;
}

void closeSocket(socket_t sock) {
    if (sock == kInvalidSocket) {
        return;
    }
#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif
}

int main() {
#ifdef _WIN32
    WSADATA wsaData{};
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }
#endif

    const socket_t serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == kInvalidSocket) {
        std::cerr << "socket() failed\n";
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(kDefaultSilPort);
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

    int reuse = 1;
    setsockopt(
        serverSocket,
        SOL_SOCKET,
        SO_REUSEADDR,
        reinterpret_cast<const char*>(&reuse),
        sizeof(reuse)
    );

    if (bind(serverSocket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) != 0) {
        std::cerr << "bind() failed on port " << kDefaultSilPort << "\n";
        closeSocket(serverSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    if (listen(serverSocket, 1) != 0) {
        std::cerr << "listen() failed\n";
        closeSocket(serverSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    std::cout << "SIL listening on 127.0.0.1:" << kDefaultSilPort << "\n";
    std::cout << "Waiting for Basilisk bridge connection...\n";

    const socket_t clientSocket = accept(serverSocket, nullptr, nullptr);
    if (clientSocket == kInvalidSocket) {
        std::cerr << "accept() failed\n";
        closeSocket(serverSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    std::cout << "Basilisk bridge connected. Running FlightSoftware loop...\n";

    FlightSoftware fsw;
    fsw.reset();

    float boot_standby_s = 0.f;
    if (!receiveBootConfig(clientSocket, &boot_standby_s)) {
        std::cerr << "receiveBootConfig failed (expected 32 B PACKET_BOOT_CONFIG)\n";
        closeSocket(clientSocket);
        closeSocket(serverSocket);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }
    fsw.setBootStandbyDuration(boot_standby_s);
    if (boot_standby_s > 0.f) {
        std::cout << "Boot Standby hold: " << boot_standby_s << " s\n";
    }

    FaultInjector injector;

    while (true) {
        SensorPacket sensors{};
        if (!receiveSensorPacket(clientSocket, sensors)) {
            std::cout << "Client disconnected.\n";
            break;
        }

        injector.apply(sensors, fsw);
        
        // Single FSW entry point — modes/control live inside step().
        const AttitudeCommand cmd = fsw.step(sensors);

        if (!sendFswStatus(clientSocket, sensors.timestamp_s, cmd.active_mode)) {
            std::cerr << "sendFswStatus failed\n";
            break;
        }
        if (cmd.apply_rw) {
            if (!sendRwTorque(clientSocket, sensors.timestamp_s, cmd.rw_torque_Nm)) {
                std::cerr << "sendRwTorque failed\n";
                break;
            }
        }
        if (cmd.apply_mtb) {
            if (!sendMtbDipole(clientSocket, sensors.timestamp_s, cmd.mtb_dipole_Am2)) {
                std::cerr << "sendMtbDipole failed\n";
                break;
            }
        }
    }

    closeSocket(clientSocket);
    closeSocket(serverSocket);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
