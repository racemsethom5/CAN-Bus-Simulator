#include "SignalParser.h"
#include <stdexcept>

// --- Generic helpers ---

uint16_t SignalParser::extract_u16_be(const CANFrame& frame, uint8_t offset) {
    return (static_cast<uint16_t>(frame.data[offset]) << 8) |
            static_cast<uint16_t>(frame.data[offset + 1]);
}

uint8_t SignalParser::extract_u8(const CANFrame& frame, uint8_t offset) {
    return frame.data[offset];
}

uint8_t SignalParser::extract_bits(uint8_t byte, uint8_t start_bit, uint8_t length) {
    uint8_t mask = (1 << length) - 1;
    return (byte >> start_bit) & mask;
}

// --- Engine signals ---

double SignalParser::parse_rpm(const CANFrame& frame) {
    if (frame.id != EngineID::RPM || frame.dlc < 2) return 0.0;
    uint16_t raw = extract_u16_be(frame, 0);
    return static_cast<double>(raw > 8000 ? 8000 : raw);
}

double SignalParser::parse_throttle(const CANFrame& frame) {
    if (frame.id != EngineID::THROTTLE || frame.dlc < 1) return 0.0;
    uint8_t raw = extract_u8(frame, 0);
    return static_cast<double>(raw > 100 ? 100 : raw);
}

double SignalParser::parse_coolant(const CANFrame& frame) {
    if (frame.id != EngineID::COOLANT || frame.dlc < 1) return 0.0;
    return static_cast<double>(extract_u8(frame, 0));
}

// --- ABS signals ---

double SignalParser::parse_vehicle_speed(const CANFrame& frame) {
    if (frame.id != ABSID::WHEEL_SPEED || frame.dlc < 4) return 0.0;
    // Average of four wheel speeds (FL, FR, RL, RR)
    uint32_t sum = frame.data[0] + frame.data[1] +
                   frame.data[2] + frame.data[3];
    return sum / 4.0;
}

uint8_t SignalParser::parse_brake_mask(const CANFrame& frame) {
    if (frame.id != ABSID::BRAKE_STATUS || frame.dlc < 1) return 0;
    return extract_u8(frame, 0) & 0x0F;  // lower 4 bits only
}

bool SignalParser::parse_abs_active(const CANFrame& frame) {
    if (frame.id != ABSID::ABS_ACTIVE || frame.dlc < 1) return false;
    return frame.data[0] != 0;
}

// --- Generic dispatch ---

bool SignalParser::parse(const CANFrame& frame,
                         const std::string& signal_name,
                         double& out_value)
{
    if (signal_name == "RPM")    { out_value = parse_rpm(frame);           return true; }
    if (signal_name == "Throttle"){ out_value = parse_throttle(frame);     return true; }
    if (signal_name == "Coolant") { out_value = parse_coolant(frame);      return true; }
    if (signal_name == "Speed")   { out_value = parse_vehicle_speed(frame);return true; }
    if (signal_name == "Brakes")  { out_value = parse_brake_mask(frame);   return true; }
    if (signal_name == "ABS")     { out_value = parse_abs_active(frame) ? 1.0 : 0.0; return true; }
    return false;
}
