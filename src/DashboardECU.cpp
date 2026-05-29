#include "DashboardECU.h"
#include <iostream>
#include <iomanip>
#include <bitset>

DashboardECU::DashboardECU(CANBus& bus)
    : ECUNode("DashboardECU", 0x7FF, bus) {}

void DashboardECU::on_update(DashUpdateCallback cb) {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    update_cb_ = std::move(cb);
}

void DashboardECU::on_receive(const CANFrame& frame) {
    if (!frame.decode()) return;

    std::string signal;
    double      value = 0.0;
    std::string unit;

    switch (frame.id) {
        case EngineID::RPM:
            if (frame.dlc >= 2) {
                value = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
                rpm_.store(value);
                signal = "RPM"; unit = "rpm";
            }
            break;

        case EngineID::THROTTLE:
            if (frame.dlc >= 1) {
                value = frame.data[0];
                throttle_.store(value);
                signal = "Throttle"; unit = "%";
            }
            break;

        case EngineID::COOLANT:
            if (frame.dlc >= 1) {
                value = frame.data[0];
                coolant_temp_.store(value);
                signal = "Coolant"; unit = "°C";
            }
            break;

        case ABSID::WHEEL_SPEED:
            if (frame.dlc >= 4) {
                // Average of all four wheels
                value = (frame.data[0] + frame.data[1] +
                         frame.data[2] + frame.data[3]) / 4.0;
                speed_.store(value);
                signal = "Speed"; unit = "km/h";
            }
            break;

        case ABSID::BRAKE_STATUS:
            if (frame.dlc >= 1) {
                brake_mask_.store(frame.data[0]);
                signal = "Brakes"; value = frame.data[0]; unit = "mask";
            }
            break;

        case ABSID::ABS_ACTIVE:
            if (frame.dlc >= 1) {
                abs_active_.store(frame.data[0] != 0);
                signal = "ABS"; value = frame.data[0]; unit = "bool";
            }
            break;

        default:
            return;
    }

    if (!signal.empty()) {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        if (update_cb_) update_cb_(signal, value, unit);
    }
}

void DashboardECU::print_status() const {
    std::cout << "=== Dashboard ===\n"
              << "  RPM     : " << std::setw(5) << rpm_.load()          << " rpm\n"
              << "  Speed   : " << std::setw(5) << speed_.load()        << " km/h\n"
              << "  Throttle: " << std::setw(5) << throttle_.load()     << " %\n"
              << "  Coolant : " << std::setw(5) << coolant_temp_.load() << " °C\n"
              << "  Brakes  : 0b" << std::bitset<4>(brake_mask_.load()) << "\n"
              << "  ABS     : " << (abs_active_.load() ? "ON" : "OFF")  << "\n"
              << "=================\n";
}
