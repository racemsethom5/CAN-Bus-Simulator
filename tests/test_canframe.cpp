#include <gtest/gtest.h>
#include "CANFrame.h"

// ── Construction ──────────────────────────────────────────────────────────

TEST(CANFrame, DefaultConstructorZeroesFields) {
    CANFrame f;
    EXPECT_EQ(f.id, 0u);
    EXPECT_EQ(f.dlc, 0u);
    EXPECT_EQ(f.crc, 0u);
    EXPECT_FALSE(f.is_remote);
    EXPECT_FALSE(f.is_error);
}

TEST(CANFrame, IDClampedTo11Bits) {
    uint8_t payload[1] = {0xAB};
    CANFrame f(0xFFF, 1, payload);
    EXPECT_EQ(f.id, 0x7FFu);
}

TEST(CANFrame, DLCClampedTo8) {
    uint8_t payload[8] = {};
    CANFrame f(0x100, 255, payload);
    EXPECT_EQ(f.dlc, 8u);
}

TEST(CANFrame, PayloadCopiedCorrectly) {
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    CANFrame f(0x100, 4, payload);
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(f.data[i], payload[i]);
}

// ── CRC ───────────────────────────────────────────────────────────────────

TEST(CANFrame, EncodeThenDecodeSucceeds) {
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    CANFrame f(0x123, 4, payload);
    f.encode();
    EXPECT_TRUE(f.decode());
}

TEST(CANFrame, TamperedDataFailsDecode) {
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    CANFrame f(0x200, 4, payload);
    f.encode();
    f.data[2] ^= 0xFF;
    EXPECT_FALSE(f.decode());
}

TEST(CANFrame, TamperedCRCFailsDecode) {
    uint8_t payload[2] = {0xAA, 0xBB};
    CANFrame f(0x300, 2, payload);
    f.encode();
    f.crc ^= 0x0001;
    EXPECT_FALSE(f.decode());
}

TEST(CANFrame, CRCDiffersForDifferentPayloads) {
    uint8_t p1[2] = {0x10, 0x20};
    uint8_t p2[2] = {0x10, 0x21};
    CANFrame f1(0x100, 2, p1);
    CANFrame f2(0x100, 2, p2);
    f1.encode(); f2.encode();
    EXPECT_NE(f1.crc, f2.crc);
}

TEST(CANFrame, CRCDiffersForDifferentIDs) {
    uint8_t payload[1] = {0x42};
    CANFrame f1(0x100, 1, payload);
    CANFrame f2(0x200, 1, payload);
    f1.encode(); f2.encode();
    EXPECT_NE(f1.crc, f2.crc);
}

// ── Edge cases ────────────────────────────────────────────────────────────

TEST(CANFrame, ZeroDLCEncodesAndDecodes) {
    CANFrame f(0x001, 0, nullptr);
    f.encode();
    EXPECT_TRUE(f.decode());
}

TEST(CANFrame, RemoteFrameHasNoPayload) {
    CANFrame f(0x7FF, 8, nullptr, /*remote=*/true);
    f.encode();
    EXPECT_TRUE(f.decode());
    EXPECT_TRUE(f.is_remote);
}

TEST(CANFrame, MaxValidFrame) {
    uint8_t payload[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    CANFrame f(0x7FF, 8, payload);
    f.encode();
    EXPECT_TRUE(f.decode());
}

TEST(CANFrame, ToStringContainsID) {
    uint8_t payload[1] = {0x42};
    CANFrame f(0x1AB, 1, payload);
    f.encode();
    std::string s = f.to_string();
    EXPECT_NE(s.find("1AB"), std::string::npos);
}
