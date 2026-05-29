#include "ECUNode.h"

ECUNode::ECUNode(const std::string& name, uint32_t base_id, CANBus& bus)
    : name_(name), base_id_(base_id), bus_(bus)
{
    // Subscribe this node to every frame on the bus
    bus_.subscribe([this](const CANFrame& frame) {
        if (state_.load() == ECUState::BUS_OFF) return;
        on_receive(frame);
    });
}

ECUNode::~ECUNode() {
    stop();
}

void ECUNode::start() {
    running_.store(true);
    state_.store(ECUState::ACTIVE);
    tx_thread_ = std::thread(&ECUNode::transmit_loop, this);
}

void ECUNode::stop() {
    running_.store(false);
    if (tx_thread_.joinable())
        tx_thread_.join();
}

void ECUNode::transmit_loop() {
    while (running_.load()) {
        if (state_.load() != ECUState::BUS_OFF)
            tick();
        std::this_thread::sleep_for(tick_interval());
    }
}

void ECUNode::send(CANFrame& frame) {
    frame.encode();
    bus_.transmit(frame);
}

void ECUNode::on_transmit_error() {
    tec_.fetch_add(8);  // CAN spec: +8 per transmit error
    bus_.record_error();
    update_state();
}

void ECUNode::on_receive_error() {
    rec_.fetch_add(1);  // CAN spec: +1 per receive error
    update_state();
}

void ECUNode::on_success() {
    // CAN spec: decrement counters by 1 on successful frame
    if (tec_ > 0) tec_.fetch_sub(1);
    if (rec_ > 0) rec_.fetch_sub(1);
    update_state();
}

void ECUNode::update_state() {
    int tec = tec_.load();
    int rec = rec_.load();

    if (tec >= 256) {
        state_.store(ECUState::BUS_OFF);
        bus_.record_error();
    } else if (tec >= 128 || rec >= 128) {
        state_.store(ECUState::ERROR_PASSIVE);
    } else if (tec >= 96 || rec >= 96) {
        state_.store(ECUState::WARNING);
    } else {
        state_.store(ECUState::ACTIVE);
    }
}

std::string ECUNode::state_str() const {
    switch (state_.load()) {
        case ECUState::INIT:          return "INIT";
        case ECUState::ACTIVE:        return "ACTIVE";
        case ECUState::WARNING:       return "WARNING";
        case ECUState::ERROR_PASSIVE: return "ERROR_PASSIVE";
        case ECUState::BUS_OFF:       return "BUS_OFF";
    }
    return "UNKNOWN";
}
