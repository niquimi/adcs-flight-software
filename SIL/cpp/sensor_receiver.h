#pragma once

#include "sensor_packet.h"
#include "sil_connection.h"

/**
 * Block until one SensorPacket is read from the SIL socket.
 * Returns false if the peer disconnects or the read fails.
 */
bool receiveSensorPacket(socket_t sock, SensorPacket& outPacket);
