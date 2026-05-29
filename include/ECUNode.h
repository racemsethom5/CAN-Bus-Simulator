#pragma once
#include "CANBus.h"
#include "CANFrame.h"
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

enum class ECUState {
    INIT,
    ACTIVE,
    WARNING,        // TEC or REC >= 96
    ERROR_PASSIVE,  // TEC or REC >= 128
    BUS_OFF         // TEC >= 256
};

class ECUNode {
public:
    ECUNode(const std::string& name, uint32_t base_id, CANBus& bus);
    virtual ~ECUNode();

    void start();   // launch transmit + receive threads
    void stop();    // graceful shutdown

    std::string name()      const { return name_; }
    ECUState    state()     const { return state_.load(); }
    std::string state_str() const;
    int         tec()       const { return tec_.load(); }
    int         rec()       const { return rec_.load(); }

protected:
    // Called by receive thread for every frame on the bus
    virtual void on_receive(const CANFrame& frame) = 0;

    // Called periodically by transmit thread — override to send frames
    virtual void tick() = 0;

    // Transmit interval — override to change rate
    virtual std::chrono::milliseconds tick_interval() const {
        return std::chrono::milliseconds(100);
    }

    // Helpers for subclasses
    void send(CANFrame& frame);        // encode + transmit
    void on_transmit_error();
    void on_receive_error();
    void on_success();

    std::string  name_;
    uint32_t     base_id_;
    CANBus&      bus_;

private:
    void transmit_loop();

    std::thread          tx_thread_;
    std::atomic<bool>    running_{false};
    std::atomic<ECUState> state_{ECUState::INIT};
    std::atomic<int>     tec_{0};  // transmit error counter
    std::atomic<int>     rec_{0};  // receive error counter

    void update_state();
};
