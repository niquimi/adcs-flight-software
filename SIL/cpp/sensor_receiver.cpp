#include "sensor_receiver.h"

#include <cstring>

bool receiveSensorPacket(socket_t sock, SensorPacket& outPacket) {
    char buffer[kSensorPacketSize]{};
    if (!recvExact(sock, buffer, kSensorPacketSize)) {
        return false;
    }
    std::memcpy(&outPacket, buffer, kSensorPacketSize);
    return true;
}
