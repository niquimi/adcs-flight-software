#pragma once

#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
constexpr socket_t kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
constexpr socket_t kInvalidSocket = -1;
#endif

/** Receive exactly `size` bytes (handles TCP fragmentation). */
bool recvExact(socket_t sock, char* buffer, std::size_t size);

/** Send exactly `size` bytes. */
bool sendExact(socket_t sock, const char* buffer, std::size_t size);

void closeSocket(socket_t sock);
