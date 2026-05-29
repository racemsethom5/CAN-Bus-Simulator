#pragma once
#include "CANFrame.h"
#include "EngineECU.h"
#include "ABSECU.h"

// Stateless decoder — maps raw CAN bytes to engineering values
class SignalParser {
public:
    // Engine signals
    static double  parse_rpm(const CANFrame& frame);       // 0–8000 rpm
    static double  parse_throttle(const CANFrame& frame);  // 0–100 %
    static double  parse_coolant(const CANFrame& frame);   // 0–150 °C

    // ABS signals
    static double  parse_vehicle_speed(const CANFrame& frame);  // avg of 4 wheels km/h
    static uint8_t parse_brake_mask(const CANFrame& frame);     // bitmask FL/FR/RL/RR
    static bool    parse_abs_active(const CANFrame& frame);

    // Generic big-endian extraction helpers
    static uint16_t extract_u16_be(const CANFrame& frame, uint8_t byte_offset);
    static uint8_t  extract_u8(const CANFrame& frame, uint8_t byte_offset);
    static uint8_t  extract_bits(uint8_t byte, uint8_t start_bit, uint8_t length);

    // Dispatch by frame ID — returns false if ID is unknown
    static bool parse(const CANFrame& frame,
                      const std::string& signal_name,
                      double& out_value);
};
