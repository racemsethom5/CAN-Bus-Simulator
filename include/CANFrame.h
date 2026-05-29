#pragma once
#include <cstdint>
#include <array>
#include <string>

// CAN 2.0A frame — 11-bit identifier, up to 8 bytes payload
struct CANFrame {
    static constexpr uint32_t MAX_ID  = 0x7FF;
    static constexpr uint8_t  MAX_DLC = 8;

    uint32_t id;           // 11-bit arbitration ID (lower = higher priority)
    uint8_t  dlc;          // data length code (0–8)
    std::array<uint8_t, 8> data{};
    uint16_t crc;          // CRC-15 computed over id + dlc + data
    bool     is_remote;    // RTR bit — request frame, no payload
    bool     is_error;     // error frame flag (set by FaultInjector)

    CANFrame() : id(0), dlc(0), crc(0), is_remote(false), is_error(false) {}
    CANFrame(uint32_t id, uint8_t dlc, const uint8_t* payload,
             bool remote = false);

    // Encode: fill crc field from current id/dlc/data
    void encode();

    // Decode: return true if stored crc matches computed crc
    bool decode() const;

    // Human-readable dump (useful for dashboard + debug)
    std::string to_string() const;

    // CRC-15 with CAN polynomial 0x4599
    static uint16_t compute_crc(const CANFrame& frame);
};
