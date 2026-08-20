#pragma once

#include "sil_connection.h"

/** Read the 32-byte boot config the plant sends once after TCP connect. */
bool receiveBootConfig(socket_t sock, float* boot_standby_s);
