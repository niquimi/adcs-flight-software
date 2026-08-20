#include "boot_config_receiver.h"

#include "command_packets.h"
#include "crc32.h"

bool receiveBootConfig(socket_t sock, float* boot_standby_s) {
    if (sock == kInvalidSocket || boot_standby_s == nullptr) {
        return false;
    }

    BootConfigCommand packet{};
    if (!recvExact(sock, reinterpret_cast<char*>(&packet), sizeof(packet))) {
        return false;
    }

    const uint32_t expected = crc32util::compute(
        &packet,
        sizeof(BootConfigCommand) - sizeof(packet.crc32)
    );
    if (packet.header.healthCheck != kCommandHealthCheck
        || packet.header.version != kPacketVersion
        || packet.header.packet_type != PACKET_BOOT_CONFIG
        || packet.crc32 != expected) {
        return false;
    }

    *boot_standby_s = packet.boot_standby_s;
    return true;
}
