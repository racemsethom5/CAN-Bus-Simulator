#include "FaultInjector.h"
#include <algorithm>

FaultInjector::FaultInjector(CANBus& bus, double fault_rate)
    : bus_(bus),
      fault_rate_(std::clamp(fault_rate, 0.0, 1.0)),
      rng_(std::random_device{}())
{}

void FaultInjector::set_fault_rate(double rate) {
    fault_rate_ = std::clamp(rate, 0.0, 1.0);
}

void FaultInjector::set_next_fault(FaultType type) {
    forced_fault_ = type;
    has_forced_   = true;
}

bool FaultInjector::inject(CANFrame& frame) {
    if (!enabled_.load()) {
        bus_.transmit(frame);
        return true;
    }

    // Decide whether to inject a fault this frame
    if (!has_forced_ && dist_(rng_) > fault_rate_) {
        bus_.transmit(frame);
        return true;
    }

    FaultType fault = has_forced_ ? forced_fault_ : pick_fault();
    has_forced_ = false;
    ++total_injected_;

    switch (fault) {
        case FaultType::BIT_FLIP:
            apply_bit_flip(frame);
            frame.is_error = true;
            ++total_corrupted_;
            bus_.transmit(frame);
            bus_.record_error();
            return true;

        case FaultType::CRC_CORRUPT:
            apply_crc_corrupt(frame);
            frame.is_error = true;
            ++total_corrupted_;
            bus_.transmit(frame);
            bus_.record_error();
            return true;

        case FaultType::FRAME_DROP:
            ++total_dropped_;
            // Frame is silently discarded — never reaches the bus
            return false;

        case FaultType::DUPLICATE:
            apply_duplicate(frame);
            bus_.transmit(frame);
            return true;

        case FaultType::NONE:
        default:
            bus_.transmit(frame);
            return true;
    }
}

FaultType FaultInjector::pick_fault() {
    switch (fault_dist_(rng_)) {
        case 0: return FaultType::BIT_FLIP;
        case 1: return FaultType::CRC_CORRUPT;
        case 2: return FaultType::FRAME_DROP;
        default: return FaultType::BIT_FLIP;
    }
}

void FaultInjector::apply_bit_flip(CANFrame& frame) {
    if (frame.dlc == 0) return;
    int byte_idx = byte_dist_(rng_) % frame.dlc;
    int bit_idx  = bit_dist_(rng_);
    frame.data[byte_idx] ^= (1 << bit_idx);
}

void FaultInjector::apply_crc_corrupt(CANFrame& frame) {
    // XOR with a non-zero pattern to guarantee mismatch
    frame.crc ^= 0x1234;
    if (frame.crc == 0) frame.crc = 0x5678;
}

void FaultInjector::apply_duplicate(CANFrame& frame) {
    // Send an extra copy before the real frame
    bus_.transmit(frame);
}

std::string FaultInjector::fault_name(FaultType t) {
    switch (t) {
        case FaultType::BIT_FLIP:    return "BIT_FLIP";
        case FaultType::CRC_CORRUPT: return "CRC_CORRUPT";
        case FaultType::FRAME_DROP:  return "FRAME_DROP";
        case FaultType::DUPLICATE:   return "DUPLICATE";
        case FaultType::NONE:        return "NONE";
    }
    return "UNKNOWN";
}
