#include "command_sender.h"

#include "command_packets.h"
#include "crc32.h"

#include <cstdint>
#include <cstring>

namespace {

uint32_t g_commandSequence = 0;

}  // namespace

bool sendRwTorque(socket_t sock, float timestamp_s, const float torque_Nm[3]) {
    if (sock == kInvalidSocket || torque_Nm == nullptr) {
        return false;
    }

    RWTorqueCommand packet{};
    packet.header.healthCheck = kCommandHealthCheck;
    packet.header.version = kPacketVersion;
    packet.header.packet_type = PACKET_RW_TORQUE_CMD;
    packet.header.sequence = ++g_commandSequence;
    packet.header.timestamp_s = timestamp_s;
    std::memcpy(packet.torque_Nm, torque_Nm, sizeof(packet.torque_Nm));

    packet.crc32 = crc32util::compute(
        &packet,
        sizeof(RWTorqueCommand) - sizeof(packet.crc32)
    );

    return sendExact(
        sock,
        reinterpret_cast<const char*>(&packet),
        kRWTorqueComandSize
    );
}

bool sendMtbDipole(socket_t sock, float timestamp_s, const float dipole_Am2[3]) {
    if (sock == kInvalidSocket || dipole_Am2 == nullptr) {
        return false;
    }

    MTBDipoleCommand packet{};
    packet.header.healthCheck = kCommandHealthCheck;
    packet.header.version = kPacketVersion;
    packet.header.packet_type = PACKET_MTB_DIPOLE_CMD;
    packet.header.sequence = ++g_commandSequence;
    packet.header.timestamp_s = timestamp_s;
    std::memcpy(packet.dipole_Am2, dipole_Am2, sizeof(packet.dipole_Am2));

    // CRC over all bytes except the trailing crc32 field (matches Python sil_protocol).
    packet.crc32 = crc32util::compute(
        &packet,
        sizeof(MTBDipoleCommand) - sizeof(packet.crc32)
    );

    return sendExact(
        sock,
        reinterpret_cast<const char*>(&packet),
        kMTBDipoleCommandSize
    );
}

bool sendFswStatus(socket_t sock, float timestamp_s, const AttitudeCommand& cmd) {
    if (sock == kInvalidSocket) {
        return false;
    }

    FswStatusCommand packet{};
    packet.header.healthCheck = kCommandHealthCheck;
    packet.header.version = kPacketVersion;
    packet.header.packet_type = PACKET_FSW_STATUS;
    packet.header.sequence = ++g_commandSequence;
    packet.header.timestamp_s = timestamp_s;
    packet.mode = static_cast<uint8_t>(cmd.active_mode);
    packet.flags = cmd.validity_flags;
    packet.tmr_mismatch_count = cmd.tmr_mismatch_count;
    packet.gyro_bias_degph = cmd.gyro_bias_degph;
    packet.health_flags = cmd.health_flags;
    packet.last_tc_opcode = cmd.last_tc_opcode;
    packet.last_tc_arg0 = cmd.last_tc_arg0;
    packet.health_event_count = cmd.health_event_count;

    packet.crc32 = crc32util::compute(
        &packet,
        sizeof(FswStatusCommand) - sizeof(packet.crc32)
    );

    return sendExact(
        sock,
        reinterpret_cast<const char*>(&packet),
        kFswStatusCommandSize
    );
}

bool sendFswAttitude(socket_t sock, float timestamp_s, const float sigma_BN[3]) {
    if (sock == kInvalidSocket || sigma_BN == nullptr) {
        return false;
    }

    FswAttitudeCommand packet{};
    packet.header.healthCheck = kCommandHealthCheck;
    packet.header.version = kPacketVersion;
    packet.header.packet_type = PACKET_FSW_ATTITUDE;
    packet.header.sequence = ++g_commandSequence;
    packet.header.timestamp_s = timestamp_s;
    std::memcpy(packet.sigma_BN, sigma_BN, sizeof(packet.sigma_BN));

    packet.crc32 = crc32util::compute(
        &packet,
        sizeof(FswAttitudeCommand) - sizeof(packet.crc32)
    );

    return sendExact(
        sock,
        reinterpret_cast<const char*>(&packet),
        kFswAttitudeCommandSize
    );
}
