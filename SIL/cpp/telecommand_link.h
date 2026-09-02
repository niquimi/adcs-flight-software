#pragma once

#include "command_packets.h"
#include "sil_connection.h"

#include <cstdint>
#include <vector>

/**
 * Second TCP port for operator telecommands. Independent of the SIL sensor
 * socket so TCs are not sent every cycle. poll() is non-blocking.
 */
class TelecommandLink {
public:
    ~TelecommandLink();

    bool listen(std::uint16_t port);
    void poll(std::vector<TelecommandPacket>& out);
    void close();

private:
    void acceptIfPending();
    void drainClient(std::vector<TelecommandPacket>& out);
    void dropClient();

    socket_t listen_ = kInvalidSocket;
    socket_t client_ = kInvalidSocket;
    std::vector<char> rx_;
};
