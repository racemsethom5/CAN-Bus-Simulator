#include "CANFrame.h"
#include <sstream>
#include <iomanip>
#include <stdexcept>

// CAN standard polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
static constexpr uint16_t CAN_CRC_POLY = 0x4599;

CANFrame::CANFrame(uint32_t id_, uint8_t dlc_, const uint8_t* payload, bool remote)
    : id(id_ & MAX_ID), dlc(dlc_ > MAX_DLC ? MAX_DLC : dlc_),
      crc(0), is_remote(remote), is_error(false)
{
    if (payload && !is_remote) {
        for (uint8_t i = 0; i < dlc; ++i)
            data[i] = payload[i];
    }
}

// Feed each bit of the message into the CRC-15 shift register.
// Input bit stream: 11-bit ID | 1-bit RTR | 4-bit DLC | dlc*8 data bits.
uint16_t CANFrame::compute_crc(const CANFrame& f)
{
    uint16_t crc_reg = 0;

    auto process_bit = [&](uint8_t bit) {
        uint16_t msb = (crc_reg >> 14) & 1;
        crc_reg = (crc_reg << 1) & 0x7FFF;
        if (msb ^ bit)
            crc_reg ^= CAN_CRC_POLY;
    };

    auto process_byte = [&](uint8_t byte, int bits = 8) {
        for (int i = bits - 1; i >= 0; --i)
            process_bit((byte >> i) & 1);
    };

    // 11-bit ID
    process_byte(static_cast<uint8_t>(f.id >> 3), 8);
    process_byte(static_cast<uint8_t>(f.id & 0x7), 3);

    // RTR + DLC (5 bits total)
    process_bit(f.is_remote ? 1 : 0);
    process_byte(f.dlc & 0x0F, 4);

    // Data bytes
    for (uint8_t i = 0; i < f.dlc; ++i)
        process_byte(f.data[i]);

    return crc_reg & 0x7FFF;
}

void CANFrame::encode()
{
    crc = compute_crc(*this);
}

bool CANFrame::decode() const
{
    return crc == compute_crc(*this);
}

std::string CANFrame::to_string() const
{
    std::ostringstream oss;
    oss << "[CAN] ID=0x" << std::hex << std::uppercase << std::setw(3)
        << std::setfill('0') << id
        << " DLC=" << std::dec << static_cast<int>(dlc)
        << " DATA=";
    for (uint8_t i = 0; i < dlc; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]);
        if (i + 1 < dlc) oss << ' ';
    }
    oss << " CRC=0x" << std::hex << std::setw(4) << std::setfill('0') << crc;
    if (is_error)  oss << " [ERROR]";
    if (is_remote) oss << " [RTR]";
    return oss.str();
}
