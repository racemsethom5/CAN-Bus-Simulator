#pragma once
#include "ECUNode.h"
#include <atomic>

// CAN IDs owned by the Engine ECU
namespace EngineID {
    constexpr uint32_t RPM      = 0x0C0;  // 2 bytes: RPM big-endian (0–8000)
    constexpr uint32_t THROTTLE = 0x0C1;  // 1 byte:  throttle % (0–100)
    constexpr uint32_t COOLANT  = 0x0C2;  // 1 byte:  coolant temp °C (0–150)
}

class EngineECU : public ECUNode {
public:
    EngineECU(CANBus& bus);

    // Setters — thread-safe, can be called from main/test
    void set_rpm(uint16_t rpm);
    void set_throttle(uint8_t pct);
    void set_coolant_temp(uint8_t temp_c);

    uint16_t rpm()          const { return rpm_.load(); }
    uint8_t  throttle()     const { return throttle_.load(); }
    uint8_t  coolant_temp() const { return coolant_temp_.load(); }

protected:
    void on_receive(const CANFrame& frame) override;
    void tick() override;
    std::chrono::milliseconds tick_interval() const override {
        return std::chrono::milliseconds(20); // 50 Hz — typical engine signal rate
    }

private:
    std::atomic<uint16_t> rpm_{0};
    std::atomic<uint8_t>  throttle_{0};
    std::atomic<uint8_t>  coolant_temp_{90};

    void send_rpm();
    void send_throttle();
    void send_coolant();
};
