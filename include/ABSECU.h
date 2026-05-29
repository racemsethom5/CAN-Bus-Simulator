#pragma once
#include "ECUNode.h"
#include <atomic>
#include <array>

// CAN IDs owned by the ABS ECU
namespace ABSID {
    constexpr uint32_t WHEEL_SPEED  = 0x1A0;  // 4 bytes: FL FR RL RR (km/h each)
    constexpr uint32_t BRAKE_STATUS = 0x1A1;  // 1 byte:  bitmask — bit N = wheel N braking
    constexpr uint32_t ABS_ACTIVE   = 0x1A2;  // 1 byte:  0=off 1=on
}

class ABSECU : public ECUNode {
public:
    ABSECU(CANBus& bus);

    // Wheel indices: 0=FL 1=FR 2=RL 3=RR
    void set_wheel_speed(int wheel, uint8_t kmh);
    void set_brake(int wheel, bool active);
    void set_abs_active(bool active);

    uint8_t wheel_speed(int wheel) const { return wheel_speeds_[wheel].load(); }
    bool    brake(int wheel)       const { return (brake_mask_.load() >> wheel) & 1; }
    bool    abs_active()           const { return abs_active_.load(); }

protected:
    void on_receive(const CANFrame& frame) override;
    void tick() override;
    std::chrono::milliseconds tick_interval() const override {
        return std::chrono::milliseconds(10); // 100 Hz — ABS needs fast updates
    }

private:
    std::array<std::atomic<uint8_t>, 4> wheel_speeds_{};
    std::atomic<uint8_t>  brake_mask_{0};
    std::atomic<bool>     abs_active_{false};

    void send_wheel_speeds();
    void send_brake_status();
    void send_abs_status();
};
