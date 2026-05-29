#include "EngineECU.h"

EngineECU::EngineECU(CANBus& bus)
    : ECUNode("EngineECU", EngineID::RPM, bus) {}

void EngineECU::set_rpm(uint16_t rpm) {
    rpm_.store(rpm > 8000 ? 8000 : rpm);
}

void EngineECU::set_throttle(uint8_t pct) {
    throttle_.store(pct > 100 ? 100 : pct);
}

void EngineECU::set_coolant_temp(uint8_t temp_c) {
    coolant_temp_.store(temp_c > 150 ? 150 : temp_c);
}

void EngineECU::tick() {
    send_rpm();
    send_throttle();
    send_coolant();
}

void EngineECU::send_rpm() {
    uint16_t r = rpm_.load();
    uint8_t payload[2] = {
        static_cast<uint8_t>(r >> 8),   // high byte
        static_cast<uint8_t>(r & 0xFF)  // low byte
    };
    CANFrame frame(EngineID::RPM, 2, payload);
    send(frame);
}

void EngineECU::send_throttle() {
    uint8_t payload[1] = { throttle_.load() };
    CANFrame frame(EngineID::THROTTLE, 1, payload);
    send(frame);
}

void EngineECU::send_coolant() {
    uint8_t payload[1] = { coolant_temp_.load() };
    CANFrame frame(EngineID::COOLANT, 1, payload);
    send(frame);
}

void EngineECU::on_receive(const CANFrame& frame) {
    // Engine ECU listens for ABS active signal to cut torque if needed
    if (frame.id == ABSID::ABS_ACTIVE && frame.dlc >= 1) {
        // Torque reduction on ABS intervention — real automotive behavior
        if (frame.data[0] == 1 && rpm_.load() > 3000)
            rpm_.store(rpm_.load() - 200);
    }
}
