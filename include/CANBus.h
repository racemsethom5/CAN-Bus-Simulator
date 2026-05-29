#pragma once
#include "CANFrame.h"
#include <queue>
#include <vector>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

enum class BusState {
    ACTIVE,   // normal operation
    WARNING,  // error counter >= 96
    BUS_OFF   // error counter >= 256 — bus is silent
};

// Comparator: lower CAN ID = higher priority
struct FramePriority {
    bool operator()(const CANFrame& a, const CANFrame& b) const {
        return a.id > b.id;
    }
};

using FrameCallback = std::function<void(const CANFrame&)>;

class CANBus {
public:
    explicit CANBus(const std::string& name = "CAN0");
    ~CANBus();

    // Transmit a frame — ECUs call this; blocks if BUS_OFF
    void transmit(const CANFrame& frame);

    // Subscribe to all frames on the bus — callback fired in receiver thread
    void subscribe(FrameCallback cb);

    // Dispatch loop — call from a dedicated thread
    void dispatch();

    // Stop the dispatch loop
    void shutdown();

    // Increment transmit/receive error counters (called by ECUs on error)
    void record_error();
    void reset_error();

    BusState    state()      const;
    std::string state_str()  const;
    std::string name()       const { return name_; }

private:
    std::string name_;

    std::priority_queue<CANFrame, std::vector<CANFrame>, FramePriority> queue_;
    std::mutex              queue_mutex_;
    std::condition_variable cv_;

    std::vector<FrameCallback> subscribers_;
    std::mutex                 sub_mutex_;

    std::atomic<int>      error_counter_{0};
    std::atomic<BusState> state_{BusState::ACTIVE};
    std::atomic<bool>     running_{false};

    void update_state();
};
