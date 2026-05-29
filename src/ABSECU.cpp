#include "ABSECU.h"

ABSECU::ABSECU(CANBus& bus)
    : ECUNode("ABSECU", ABSID::WHEEL_SPEED, bus)
{
    for (auto& ws : wheel_speeds_)
        ws.store(0);
}

void ABSECU::set_wheel_speed(int wheel, uint8_t kmh) {
    if (wheel < 0 || wheel > 3) return;
    wheel_speeds_[wheel].store(kmh);
}

void ABSECU::set_brake(int wheel, bool active) {
    if (wheel < 0 || wheel > 3) return;
    uint8_t mask = brake_mask_.load();
    if (active)
        mask |= (1 << wheel);
    else
        mask &= ~(1 << wheel);
    brake_mask_.store(mask);
}

void ABSECU::set_abs_active(bool active) {
    abs_active_.store(active);
}

void ABSECU::tick() {
    send_wheel_speeds();
    send_brake_status();
    send_abs_status();
}

void ABSECU::send_wheel_speeds() {
    uint8_t payload[4] = {
        wheel_speeds_[0].load(),  // FL
        wheel_speeds_[1].load(),  // FR
        wheel_speeds_[2].load(),  // RL
        wheel_speeds_[3].load()   // RR
    };
    CANFrame frame(ABSID::WHEEL_SPEED, 4, payload);
    send(frame);
}

void ABSECU::send_brake_status() {
    uint8_t payload[1] = { brake_mask_.load() };
    CANFrame frame(ABSID::BRAKE_STATUS, 1, payload);
    send(frame);
}

void ABSECU::send_abs_status() {
    uint8_t payload[1] = { abs_active_.load() ? uint8_t(1) : uint8_t(0) };
    CANFrame frame(ABSID::ABS_ACTIVE, 1, payload);
    send(frame);
}

void ABSECU::on_receive(const CANFrame& frame) {
    // ABS monitors engine RPM to detect wheel slip
    if (frame.id == EngineID::RPM && frame.dlc >= 2) {
        uint16_t rpm = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        // Detect slip: if engine RPM suggests > 200 km/h but wheels disagree
        uint8_t max_wheel = 0;
        for (int i = 0; i < 4; ++i) {
            uint8_t ws = wheel_speeds_[i].load();
            if (ws > max_wheel) max_wheel = ws;
        }
        bool slip = rpm > 6000 && max_wheel < 50;
        if (slip != abs_active_.load())
            abs_active_.store(slip);
    }
}
