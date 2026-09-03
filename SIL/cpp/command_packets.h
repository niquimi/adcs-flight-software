#pragma once

#include <cstddef>
#include <cstdint>

#pragma pack(push, 1)

struct CommandHeader {
    uint32_t healthCheck;
    uint16_t version;
    uint16_t packet_type;
    uint32_t sequence;
    float timestamp_s;
};

struct RWTorqueCommand {
    CommandHeader header;
    float torque_Nm[3];
    uint32_t crc32;
};

struct MTBDipoleCommand {
    CommandHeader header;
    float dipole_Am2[3];
    uint32_t crc32;
};

struct FswStatusCommand {
    CommandHeader header;
    uint8_t mode;
    uint8_t flags;

    uint16_t tmr_mismatch_count;
    float gyro_bias_degph;
    uint8_t health_flags;
    uint8_t last_tc_opcode;
    uint8_t last_tc_arg0;
    uint8_t health_event_count;

    uint32_t crc32;
};

struct BootConfigCommand {
    CommandHeader header;
    float boot_standby_s;
    float reserved[2];
    uint32_t crc32;
};

struct FswAttitudeCommand {
    CommandHeader header;
    float sigma_BN[3];
    uint32_t crc32;
};

struct TelecommandPacket {
    CommandHeader header;
    uint8_t opcode;   // TelecommandOpcode 0..9
    uint8_t arg0;
    uint8_t arg1;
    uint8_t reserved[9];
    uint32_t crc32;
};
static_assert(sizeof(TelecommandPacket) == 32);

enum TelecommandOpcode : uint8_t {
    TC_IDLE = 0,
    TC_FORCE_MODE = 1,
    TC_CLEAR_FORCE = 2,
    TC_INJECT_FAULT = 3,  // arg0: 1 = ModeId replica, 2 = SOC 0.10
    TC_CLEAR_FAULTS = 4,
    TC_SET_POINTING_TARGET = 5,
    TC_RESET_ESTIMATOR = 6,
    TC_SET_GAINS = 7,
    TC_SET_THRESHOLDS = 8,
    TC_RESET_FSW = 9,
};

#pragma pack(pop)

constexpr uint32_t kCommandHealthCheck = 0x53494C43; // "SILC" in ASCII for synchronization check
constexpr uint16_t kPacketVersion = 1;

enum PacketType : uint16_t {
    PACKET_RW_TORQUE_CMD = 1,
    PACKET_MTB_DIPOLE_CMD = 2,
    PACKET_FSW_STATUS = 3,
    PACKET_BOOT_CONFIG = 4,
    PACKET_TELECOMMAND = 5,
    PACKET_FSW_ATTITUDE = 6,
};

static_assert(sizeof(CommandHeader) == 16);

static_assert(sizeof(RWTorqueCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(MTBDipoleCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(FswStatusCommand) == sizeof(CommandHeader) + 12 + 4);
static_assert(sizeof(BootConfigCommand) == sizeof(CommandHeader) + 12 + 4);
static_assert(sizeof(FswAttitudeCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(float) == 4);

constexpr std::size_t kRWTorqueComandSize = sizeof(RWTorqueCommand);
constexpr std::size_t kMTBDipoleCommandSize = sizeof(MTBDipoleCommand);
constexpr std::size_t kFswStatusCommandSize = sizeof(FswStatusCommand);
constexpr std::size_t kFswAttitudeCommandSize = sizeof(FswAttitudeCommand);
constexpr std::size_t kTelecommandPacketSize = sizeof(TelecommandPacket);

constexpr std::size_t kCommandHeaderSize = sizeof(CommandHeader);