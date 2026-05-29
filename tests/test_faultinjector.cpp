#include <gtest/gtest.h>
#include "FaultInjector.h"
#include <atomic>

// Helper — build a valid encoded frame
static CANFrame make_frame(uint32_t id = 0x100) {
    uint8_t p[4] = {0x01, 0x02, 0x03, 0x04};
    CANFrame f(id, 4, p);
    f.encode();
    return f;
}

// ── Written by user (3 tests) ─────────────────────────────────────────────

TEST(FaultInjector, DisabledNeverInjects) {
    CANBus bus("TEST");
    FaultInjector fi(bus, 1.0);  // 100% fault rate
    fi.disable();

    std::atomic<int> count{0};
    bus.subscribe([&](const CANFrame&) { ++count; });
    std::thread t([&bus]{ bus.dispatch(); });

    for (int i = 0; i < 10; ++i) {
        CANFrame f = make_frame();
        fi.inject(f);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bus.shutdown();
    t.join();

    EXPECT_EQ(fi.total_injected(), 0u);
    EXPECT_EQ(count.load(), 10);
}

TEST(FaultInjector, FrameDropNeverReachesBus) {
    CANBus bus("TEST");
    FaultInjector fi(bus, 0.0);
    std::atomic<int> count{0};
    bus.subscribe([&](const CANFrame&) { ++count; });
    std::thread t([&bus]{ bus.dispatch(); });

    fi.set_next_fault(FaultType::FRAME_DROP);
    CANFrame f = make_frame();
    bool delivered = fi.inject(f);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    bus.shutdown();
    t.join();

    EXPECT_FALSE(delivered);
    EXPECT_EQ(count.load(), 0);
    EXPECT_EQ(fi.total_dropped(), 1u);
}

TEST(FaultInjector, CRCCorruptMakesFrameInvalid) {
    CANBus bus("TEST");
    FaultInjector fi(bus, 0.0);
    std::atomic<int> received{0};
    bus.subscribe([&](const CANFrame& frame) {
        if (frame.is_error) ++received;
    });
    std::thread t([&bus]{ bus.dispatch(); });

    fi.set_next_fault(FaultType::CRC_CORRUPT);
    CANFrame f = make_frame();
    fi.inject(f);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    bus.shutdown();
    t.join();

    EXPECT_EQ(fi.total_corrupted(), 1u);
}

// ── Written by Claude (3 tests) ───────────────────────────────────────────

TEST(FaultInjector, BitFlipChangesAtLeastOneBit) {
    CANBus bus("TEST");
    FaultInjector fi(bus, 0.0);

    CANFrame original = make_frame();
    CANFrame mutated  = original;

    fi.set_next_fault(FaultType::BIT_FLIP);
    fi.inject(mutated);

    // At least one data byte must differ
    bool any_diff = false;
    for (int i = 0; i < original.dlc; ++i)
        if (original.data[i] != mutated.data[i]) { any_diff = true; break; }

    EXPECT_TRUE(any_diff);
    EXPECT_EQ(fi.total_corrupted(), 1u);
    bus.shutdown();
}

TEST(FaultInjector, FaultRateZeroNeverInjects) {
    CANBus bus("TEST");
    FaultInjector fi(bus, 0.0);
    std::thread t([&bus]{ bus.dispatch(); });

    for (int i = 0; i < 50; ++i) {
        CANFrame f = make_frame();
        fi.inject(f);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bus.shutdown();
    t.join();

    EXPECT_EQ(fi.total_injected(), 0u);
}

TEST(FaultInjector, FaultNameStringsCorrect) {
    EXPECT_EQ(FaultInjector::fault_name(FaultType::BIT_FLIP),    "BIT_FLIP");
    EXPECT_EQ(FaultInjector::fault_name(FaultType::CRC_CORRUPT), "CRC_CORRUPT");
    EXPECT_EQ(FaultInjector::fault_name(FaultType::FRAME_DROP),  "FRAME_DROP");
    EXPECT_EQ(FaultInjector::fault_name(FaultType::DUPLICATE),   "DUPLICATE");
    EXPECT_EQ(FaultInjector::fault_name(FaultType::NONE),        "NONE");
}
