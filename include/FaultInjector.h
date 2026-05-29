#pragma once
#include "CANFrame.h"
#include "CANBus.h"
#include <random>
#include <atomic>
#include <string>

enum class FaultType {
    BIT_FLIP,       // randomly flip one bit in the payload
    CRC_CORRUPT,    // overwrite CRC with a bad value
    FRAME_DROP,     // silently discard the frame
    DUPLICATE,      // transmit frame twice
    NONE
};

class FaultInjector {
public:
    explicit FaultInjector(CANBus& bus, double fault_rate = 0.05);

    // Wrap a frame through fault injection before it reaches the bus.
    // Returns false if frame was dropped.
    bool inject(CANFrame& frame);

    // Force a specific fault on next inject() call
    void set_next_fault(FaultType type);

    // Enable / disable injection globally
    void enable()  { enabled_.store(true); }
    void disable() { enabled_.store(false); }
    bool enabled() const { return enabled_.load(); }

    // Fault rate: 0.0 = never, 1.0 = always
    void set_fault_rate(double rate);

    // Statistics
    uint32_t total_injected()  const { return total_injected_.load(); }
    uint32_t total_dropped()   const { return total_dropped_.load(); }
    uint32_t total_corrupted() const { return total_corrupted_.load(); }

    static std::string fault_name(FaultType t);

private:
    CANBus&             bus_;
    double              fault_rate_;
    std::atomic<bool>   enabled_{true};
    FaultType           forced_fault_{FaultType::NONE};
    bool                has_forced_{false};

    std::mt19937                          rng_;
    std::uniform_real_distribution<double> dist_{0.0, 1.0};
    std::uniform_int_distribution<int>    byte_dist_{0, 7};
    std::uniform_int_distribution<int>    bit_dist_{0, 7};
    std::uniform_int_distribution<int>    fault_dist_{0, 2};

    std::atomic<uint32_t> total_injected_{0};
    std::atomic<uint32_t> total_dropped_{0};
    std::atomic<uint32_t> total_corrupted_{0};

    FaultType pick_fault();
    void apply_bit_flip(CANFrame& frame);
    void apply_crc_corrupt(CANFrame& frame);
    void apply_duplicate(CANFrame& frame);
};
