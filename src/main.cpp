#include "CANBus.h"
#include "EngineECU.h"
#include "ABSECU.h"
#include "DashboardECU.h"
#include "FaultInjector.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;

static void print_separator(const std::string& label) {
    std::cout << "\n--- " << label << " ---\n";
}

int main() {
    std::cout << "CAN Bus Simulator — 3-ECU Live Demo\n";
    std::cout << "=====================================\n";

    // ── Bus ──────────────────────────────────────────────────────────────
    CANBus bus("CAN0");

    // ── ECU nodes ────────────────────────────────────────────────────────
    EngineECU    engine(bus);
    ABSECU       abs(bus);
    DashboardECU dash(bus);

    // Live signal callback — prints every update to stdout
    dash.on_update([](const std::string& signal, double value,
                      const std::string& unit) {
        std::cout << "  [DASH] " << std::left << std::setw(10) << signal
                  << " = " << std::right << std::setw(7)
                  << std::fixed << std::setprecision(1) << value
                  << " " << unit << "\n";
    });

    // ── Fault injector ────────────────────────────────────────────────────
    FaultInjector injector(bus, 0.0);  // start with faults off
    injector.disable();

    // ── Dispatch thread (runs the bus) ────────────────────────────────────
    std::thread bus_thread([&bus] { bus.dispatch(); });

    // ── Start all ECUs ────────────────────────────────────────────────────
    engine.start();
    abs.start();
    dash.start();

    // ═════════════════════════════════════════════════════════════════════
    // Phase 1: Engine idle
    // ═════════════════════════════════════════════════════════════════════
    print_separator("Phase 1: Engine idle");
    engine.set_rpm(800);
    engine.set_throttle(5);
    engine.set_coolant_temp(90);

    for (int i = 0; i < 4; ++i) abs.set_wheel_speed(i, 0);

    std::this_thread::sleep_for(500ms);
    dash.print_status();

    // ═════════════════════════════════════════════════════════════════════
    // Phase 2: Acceleration
    // ═════════════════════════════════════════════════════════════════════
    print_separator("Phase 2: Acceleration");

    for (uint16_t rpm = 1000; rpm <= 5000; rpm += 500) {
        engine.set_rpm(rpm);
        engine.set_throttle(static_cast<uint8_t>(rpm / 80));
        uint8_t speed = static_cast<uint8_t>(rpm / 50);
        for (int i = 0; i < 4; ++i) abs.set_wheel_speed(i, speed);
        std::this_thread::sleep_for(150ms);
    }

    std::this_thread::sleep_for(300ms);
    dash.print_status();

    // ═════════════════════════════════════════════════════════════════════
    // Phase 3: Hard braking — ABS kicks in
    // ═════════════════════════════════════════════════════════════════════
    print_separator("Phase 3: Hard braking — ABS event");

    engine.set_throttle(0);
    for (int i = 0; i < 4; ++i) abs.set_brake(i, true);
    abs.set_abs_active(true);

    // Wheels lock — speed drops faster than engine RPM
    for (uint8_t speed = 100; speed > 0; speed -= 20) {
        for (int i = 0; i < 4; ++i) abs.set_wheel_speed(i, speed);
        engine.set_rpm(engine.rpm() > 300 ? engine.rpm() - 300 : 800);
        std::this_thread::sleep_for(150ms);
    }

    std::this_thread::sleep_for(300ms);
    dash.print_status();

    // ═════════════════════════════════════════════════════════════════════
    // Phase 4: Fault injection demo
    // ═════════════════════════════════════════════════════════════════════
    print_separator("Phase 4: Fault injection");
    injector.enable();
    injector.set_fault_rate(0.8);  // 80% fault rate for demo visibility

    engine.set_rpm(2000);
    engine.set_throttle(25);
    for (int i = 0; i < 4; ++i) abs.set_brake(i, false);
    abs.set_abs_active(false);

    // Inject specific faults in sequence
    std::cout << "  Injecting BIT_FLIP...\n";
    injector.set_next_fault(FaultType::BIT_FLIP);
    std::this_thread::sleep_for(100ms);

    std::cout << "  Injecting CRC_CORRUPT...\n";
    injector.set_next_fault(FaultType::CRC_CORRUPT);
    std::this_thread::sleep_for(100ms);

    std::cout << "  Injecting FRAME_DROP...\n";
    injector.set_next_fault(FaultType::FRAME_DROP);
    std::this_thread::sleep_for(100ms);

    std::cout << "\n  Bus state  : " << bus.state_str() << "\n";
    std::cout << "  Faults injected : " << injector.total_injected()  << "\n";
    std::cout << "  Frames dropped  : " << injector.total_dropped()   << "\n";
    std::cout << "  Frames corrupted: " << injector.total_corrupted() << "\n";

    // ═════════════════════════════════════════════════════════════════════
    // Shutdown
    // ═════════════════════════════════════════════════════════════════════
    print_separator("Shutdown");
    injector.disable();

    engine.stop();
    abs.stop();
    dash.stop();
    bus.shutdown();
    bus_thread.join();

    std::cout << "All ECUs stopped. Bus offline.\n";
    return 0;
}
