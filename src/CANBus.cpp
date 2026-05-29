#include "CANBus.h"
#include <stdexcept>

CANBus::CANBus(const std::string& name) : name_(name) {}

CANBus::~CANBus() {
    shutdown();
}

void CANBus::transmit(const CANFrame& frame) {
    if (state_.load() == BusState::BUS_OFF)
        return; // silently drop — bus is offline

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push(frame);
    }
    cv_.notify_one();
}

void CANBus::subscribe(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    subscribers_.push_back(std::move(cb));
}

void CANBus::dispatch() {
    running_.store(true);

    while (running_.load()) {
        CANFrame frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            // Wait until a frame is available or shutdown is requested
            cv_.wait(lock, [this] {
                return !queue_.empty() || !running_.load();
            });

            if (!running_.load() && queue_.empty())
                break;

            frame = queue_.top();
            queue_.pop();
        }

        // Validate CRC — record error on corrupt frame
        if (!frame.is_error && !frame.decode()) {
            record_error();
            continue;
        }

        // Fan out to all subscribers
        std::vector<FrameCallback> cbs;
        {
            std::lock_guard<std::mutex> lock(sub_mutex_);
            cbs = subscribers_;
        }
        for (auto& cb : cbs)
            cb(frame);
    }
}

void CANBus::shutdown() {
    running_.store(false);
    cv_.notify_all();
}

void CANBus::record_error() {
    ++error_counter_;
    update_state();
}

void CANBus::reset_error() {
    if (error_counter_ > 0)
        --error_counter_;
    update_state();
}

void CANBus::update_state() {
    int count = error_counter_.load();
    if (count >= 256)
        state_.store(BusState::BUS_OFF);
    else if (count >= 96)
        state_.store(BusState::WARNING);
    else
        state_.store(BusState::ACTIVE);
}

BusState CANBus::state() const {
    return state_.load();
}

std::string CANBus::state_str() const {
    switch (state_.load()) {
        case BusState::ACTIVE:  return "ACTIVE";
        case BusState::WARNING: return "WARNING";
        case BusState::BUS_OFF: return "BUS_OFF";
    }
    return "UNKNOWN";
}
