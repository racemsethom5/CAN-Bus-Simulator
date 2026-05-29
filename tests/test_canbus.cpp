#include <gtest/gtest.h>
#include "CANBus.h"
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>

using namespace std::chrono_literals;

// ── Priority ordering ─────────────────────────────────────────────────────

TEST(CANBus, LowerIDDeliveredFirst) {
    CANBus bus("TEST");
    std::vector<uint32_t> received;

    bus.subscribe([&](const CANFrame& f) {
        received.push_back(f.id);
    });

    // Transmit high-ID frames first, then low — bus must reorder
    uint8_t p[1] = {0x01};
    CANFrame hi(0x7FF, 1, p); hi.encode();
    CANFrame lo(0x001, 1, p); lo.encode();
    CANFrame mid(0x100, 1, p); mid.encode();

    bus.transmit(hi);
    bus.transmit(lo);
    bus.transmit(mid);

    std::thread t([&bus]{ bus.dispatch(); });
    std::this_thread::sleep_for(50ms);
    bus.shutdown();
    t.join();

    ASSERT_GE(received.size(), 3u);
    EXPECT_LT(received[0], received[1]);
    EXPECT_LT(received[1], received[2]);
}

// ── Thread safety ─────────────────────────────────────────────────────────

TEST(CANBus, ConcurrentTransmitNoDataRace) {
    CANBus bus("TEST");
    std::atomic<int> count{0};

    bus.subscribe([&](const CANFrame&) { ++count; });

    std::thread dispatcher([&bus]{ bus.dispatch(); });

    // 4 threads each transmit 25 frames — 100 total
    std::vector<std::thread> senders;
    for (int t = 0; t < 4; ++t) {
        senders.emplace_back([&bus, t] {
            uint8_t p[1] = {static_cast<uint8_t>(t)};
            for (int i = 0; i < 25; ++i) {
                CANFrame f(static_cast<uint32_t>(0x100 + t), 1, p);
                f.encode();
                bus.transmit(f);
                std::this_thread::sleep_for(1ms);
            }
        });
    }
    for (auto& s : senders) s.join();

    std::this_thread::sleep_for(100ms);
    bus.shutdown();
    dispatcher.join();

    EXPECT_EQ(count.load(), 100);
}

// ── Bus-off state ─────────────────────────────────────────────────────────

TEST(CANBus, BusOffAfter256Errors) {
    CANBus bus("TEST");
    for (int i = 0; i < 256; ++i)
        bus.record_error();
    EXPECT_EQ(bus.state(), BusState::BUS_OFF);
}

TEST(CANBus, WarningAfter96Errors) {
    CANBus bus("TEST");
    for (int i = 0; i < 96; ++i)
        bus.record_error();
    EXPECT_EQ(bus.state(), BusState::WARNING);
}

TEST(CANBus, TransmitDroppedWhenBusOff) {
    CANBus bus("TEST");
    std::atomic<int> count{0};
    bus.subscribe([&](const CANFrame&) { ++count; });

    for (int i = 0; i < 256; ++i) bus.record_error();

    uint8_t p[1] = {0x01};
    CANFrame f(0x100, 1, p); f.encode();
    bus.transmit(f);  // must be silently dropped

    std::thread t([&bus]{ bus.dispatch(); });
    std::this_thread::sleep_for(30ms);
    bus.shutdown();
    t.join();

    EXPECT_EQ(count.load(), 0);
}

// ── Corrupt frame dropped by dispatcher ───────────────────────────────────

TEST(CANBus, CorruptFrameNotDeliveredToSubscribers) {
    CANBus bus("TEST");
    std::atomic<int> count{0};
    bus.subscribe([&](const CANFrame&) { ++count; });

    uint8_t p[2] = {0xAA, 0xBB};
    CANFrame f(0x200, 2, p);
    f.encode();
    f.crc ^= 0x1234;  // corrupt after encoding
    bus.transmit(f);

    std::thread t([&bus]{ bus.dispatch(); });
    std::this_thread::sleep_for(30ms);
    bus.shutdown();
    t.join();

    EXPECT_EQ(count.load(), 0);
}

TEST(CANBus, StateStringMatchesState) {
    CANBus bus("TEST");
    EXPECT_EQ(bus.state_str(), "ACTIVE");
    for (int i = 0; i < 96; ++i) bus.record_error();
    EXPECT_EQ(bus.state_str(), "WARNING");
}
