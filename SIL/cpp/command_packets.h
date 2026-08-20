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
    uint8_t reserved[11];
    uint32_t crc32;
};

struct BootConfigCommand {
    CommandHeader header;
    float boot_standby_s;
    float reserved[2];
    uint32_t crc32;
};

#pragma pack(pop)

constexpr uint32_t kCommandHealthCheck = 0x53494C43; // "SILC" in ASCII for synchronization check
constexpr uint16_t kPacketVersion = 1;

enum PacketType : uint16_t {
    PACKET_RW_TORQUE_CMD = 1,
    PACKET_MTB_DIPOLE_CMD = 2,
    PACKET_FSW_STATUS = 3,
    PACKET_BOOT_CONFIG = 4,
};

static_assert(sizeof(CommandHeader) == 16);

static_assert(sizeof(RWTorqueCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(MTBDipoleCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(FswStatusCommand) == sizeof(CommandHeader) + 12 + 4);
static_assert(sizeof(BootConfigCommand) == sizeof(CommandHeader) + 12 + 4);

static_assert(sizeof(float) == 4);

constexpr std::size_t kRWTorqueComandSize = sizeof(RWTorqueCommand);
constexpr std::size_t kMTBDipoleCommandSize = sizeof(MTBDipoleCommand);
constexpr std::size_t kFswStatusCommandSize = sizeof(FswStatusCommand);

constexpr std::size_t kCommandHeaderSize = sizeof(CommandHeader);