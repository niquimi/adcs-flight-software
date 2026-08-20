#pragma once

#include "sil_connection.h"
#include "types.h"

/**
 * Pack and send an RWTorqueCommand (32 bytes) on the shared SIL socket.
 * Skeleton: no packet build / send until FSW wiring is ready.
 */
bool sendRwTorque(socket_t sock, float timestamp_s, const float torque_Nm[3]);

/**
 * Pack and send an MTBDipoleCommand (32 bytes) on the shared SIL socket.
 * Skeleton: no packet build / send until FSW wiring is ready.
 */
bool sendMtbDipole(socket_t sock, float timestamp_s, const float dipole_Am2[3]);

/** Pack and send an FswStatusCommand (32 bytes) with the active FSW mode. */
bool sendFswStatus(socket_t sock, float timestamp_s, ModeId mode);
