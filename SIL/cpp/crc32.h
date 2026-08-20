#pragma once

#include<array>
#include<cstddef>
#include<cstdint>

namespace crc32util {

namespace detail {

constexpr std::array<uint32_t, 256> makeTable() {
    std::array<uint32_t, 256> result{};
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        result[i] = c;
    }
    return result;
}

}

inline constexpr std::array<uint32_t, 256> kTable = detail::makeTable();

inline uint32_t compute(const void* data, std::size_t length) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < length; i++) {
        crc = kTable[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

}