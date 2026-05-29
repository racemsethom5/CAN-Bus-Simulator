#include <gtest/gtest.h>
#include "SignalParser.h"

// ── RPM ───────────────────────────────────────────────────────────────────

TEST(SignalParser, RPMZero) {
    uint8_t p[2] = {0x00, 0x00};
    CANFrame f(EngineID::RPM, 2, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_rpm(f), 0.0);
}

TEST(SignalParser, RPMMaxValue) {
    uint8_t p[2] = {0x1F, 0x40};  // 8000 = 0x1F40
    CANFrame f(EngineID::RPM, 2, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_rpm(f), 8000.0);
}

TEST(SignalParser, RPMOverflowClampedTo8000) {
    uint8_t p[2] = {0xFF, 0xFF};  // 65535 > 8000
    CANFrame f(EngineID::RPM, 2, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_rpm(f), 8000.0);
}

TEST(SignalParser, RPMWrongIDReturnsZero) {
    uint8_t p[2] = {0x1F, 0x40};
    CANFrame f(0x999, 2, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_rpm(f), 0.0);
}

TEST(SignalParser, RPMBigEndianByteOrder) {
    // 3000 = 0x0BB8 → high byte 0x0B, low byte 0xB8
    uint8_t p[2] = {0x0B, 0xB8};
    CANFrame f(EngineID::RPM, 2, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_rpm(f), 3000.0);
}

// ── Throttle ──────────────────────────────────────────────────────────────

TEST(SignalParser, ThrottleFullRange) {
    for (uint8_t pct : {0, 25, 50, 75, 100}) {
        uint8_t p[1] = {pct};
        CANFrame f(EngineID::THROTTLE, 1, p); f.encode();
        EXPECT_DOUBLE_EQ(SignalParser::parse_throttle(f), static_cast<double>(pct));
    }
}

TEST(SignalParser, ThrottleOverflowClamped) {
    uint8_t p[1] = {200};
    CANFrame f(EngineID::THROTTLE, 1, p); f.encode();
    EXPECT_LE(SignalParser::parse_throttle(f), 100.0);
}

// ── ABS wheel speed ───────────────────────────────────────────────────────

TEST(SignalParser, VehicleSpeedAverageOfFourWheels) {
    uint8_t p[4] = {100, 100, 100, 100};
    CANFrame f(ABSID::WHEEL_SPEED, 4, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_vehicle_speed(f), 100.0);
}

TEST(SignalParser, VehicleSpeedUnevenWheels) {
    uint8_t p[4] = {80, 100, 80, 100};
    CANFrame f(ABSID::WHEEL_SPEED, 4, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_vehicle_speed(f), 90.0);
}

TEST(SignalParser, VehicleSpeedAllZero) {
    uint8_t p[4] = {0, 0, 0, 0};
    CANFrame f(ABSID::WHEEL_SPEED, 4, p); f.encode();
    EXPECT_DOUBLE_EQ(SignalParser::parse_vehicle_speed(f), 0.0);
}

// ── Brake mask ────────────────────────────────────────────────────────────

TEST(SignalParser, BrakeMaskAllOff) {
    uint8_t p[1] = {0x00};
    CANFrame f(ABSID::BRAKE_STATUS, 1, p); f.encode();
    EXPECT_EQ(SignalParser::parse_brake_mask(f), 0u);
}

TEST(SignalParser, BrakeMaskAllOn) {
    uint8_t p[1] = {0x0F};
    CANFrame f(ABSID::BRAKE_STATUS, 1, p); f.encode();
    EXPECT_EQ(SignalParser::parse_brake_mask(f), 0x0Fu);
}

TEST(SignalParser, BrakeMaskUpperBitsIgnored) {
    uint8_t p[1] = {0xFF};
    CANFrame f(ABSID::BRAKE_STATUS, 1, p); f.encode();
    EXPECT_EQ(SignalParser::parse_brake_mask(f), 0x0Fu);
}

// ── ABS active ────────────────────────────────────────────────────────────

TEST(SignalParser, ABSActiveTrue) {
    uint8_t p[1] = {0x01};
    CANFrame f(ABSID::ABS_ACTIVE, 1, p); f.encode();
    EXPECT_TRUE(SignalParser::parse_abs_active(f));
}

TEST(SignalParser, ABSActiveFalse) {
    uint8_t p[1] = {0x00};
    CANFrame f(ABSID::ABS_ACTIVE, 1, p); f.encode();
    EXPECT_FALSE(SignalParser::parse_abs_active(f));
}

// ── Generic dispatch ──────────────────────────────────────────────────────

TEST(SignalParser, GenericDispatchKnownSignal) {
    uint8_t p[2] = {0x0B, 0xB8};  // 3000 RPM
    CANFrame f(EngineID::RPM, 2, p); f.encode();
    double val = 0.0;
    EXPECT_TRUE(SignalParser::parse(f, "RPM", val));
    EXPECT_DOUBLE_EQ(val, 3000.0);
}

TEST(SignalParser, GenericDispatchUnknownSignalReturnsFalse) {
    uint8_t p[1] = {0x00};
    CANFrame f(0x100, 1, p); f.encode();
    double val = 0.0;
    EXPECT_FALSE(SignalParser::parse(f, "UNKNOWN", val));
}
