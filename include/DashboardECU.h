#pragma once
#include "ECUNode.h"
#include "EngineECU.h"
#include "ABSECU.h"
#include <atomic>
#include <string>
#include <functional>

// Callback fired whenever the dashboard receives a decoded signal update
using DashUpdateCallback = std::function<void(const std::string& signal,
                                               double value,
                                               const std::string& unit)>;

class DashboardECU : public ECUNode {
public:
    DashboardECU(CANBus& bus);

    // Register a callback for live signal updates (e.g. UI refresh)
    void on_update(DashUpdateCallback cb);

    // Latest decoded values — read by display / tests
    double  rpm()          const { return rpm_.load(); }
    double  speed()        const { return speed_.load(); }
    double  throttle()     const { return throttle_.load(); }
    double  coolant_temp() const { return coolant_temp_.load(); }
    uint8_t brake_mask()   const { return brake_mask_.load(); }
    bool    abs_active()   const { return abs_active_.load(); }

    // Print current state to stdout
    void print_status() const;

protected:
    void on_receive(const CANFrame& frame) override;
    void tick() override {}  // dashboard is receive-only — no transmit

private:
    std::atomic<double>  rpm_{0.0};
    std::atomic<double>  speed_{0.0};
    std::atomic<double>  throttle_{0.0};
    std::atomic<double>  coolant_temp_{0.0};
    std::atomic<uint8_t> brake_mask_{0};
    std::atomic<bool>    abs_active_{false};

    DashUpdateCallback   update_cb_;
    std::mutex           cb_mutex_;
};
