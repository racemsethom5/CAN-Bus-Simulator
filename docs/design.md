# Architecture & Design Decisions

## Overview

This simulator models a real CAN 2.0A network with three ECU nodes communicating
over a shared bus. The design prioritizes accuracy to the CAN specification over
simplicity — every component maps to a real automotive concept.

---

## CAN 2.0A Frame Choice

**Decision:** CAN 2.0A (11-bit identifier) over CAN 2.0B (29-bit extended).

**Reason:** The 11-bit standard frame covers the majority of powertrain ECU
communication (engine, ABS, dashboard) in production vehicles. BMW and Bosch
use CAN 2.0A for high-speed powertrain buses (500 kbps) and reserve 2.0B for
lower-speed body/chassis buses. For a simulator targeting powertrain signals,
2.0A is the correct scope.

---

## CRC-15 Implementation

**Polynomial:** `x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1` → `0x4599`

**Bit stream:** 11-bit ID | RTR | 4-bit DLC | data bytes (big-endian, MSB first)

**Why bit-level:** The CAN CRC operates on the raw bit stream including stuffed
bits in hardware. The software implementation processes the logical message field
(pre-stuffing) which is standard practice for simulation and test tooling.

**Validation:** Any tampered byte or CRC field change causes `decode()` to return
`false`. The bus dispatcher silently drops corrupt frames and increments the error
counter — matching ISO 11898-1 behavior.

---

## Thread Model

```
main thread
  │
  ├─ CANBus::dispatch()   [bus thread]
  │    └─ dequeues frames → calls all subscriber callbacks
  │
  ├─ ECUNode::transmit_loop()  [per ECU — N threads]
  │    └─ calls tick() every tick_interval()
  │         └─ builds CANFrame → encode() → bus.transmit()
  │
  └─ subscriber callbacks fire on the bus thread
       └─ on_receive() implementations must be thread-safe
```

**Synchronization:**
- `CANBus::queue_` protected by `queue_mutex_` + `condition_variable`
- `CANBus::subscribers_` protected by `sub_mutex_` (copied before dispatch)
- All ECU signal fields are `std::atomic<T>` — lock-free reads from any thread

---

## Priority Queue

CAN arbitration is non-destructive: the node with the lowest ID wins the bus.
The simulator models this with `std::priority_queue` using a custom comparator:

```cpp
struct FramePriority {
    bool operator()(const CANFrame& a, const CANFrame& b) const {
        return a.id > b.id;  // lower ID = higher priority
    }
};
```

This means `0x001` is always dispatched before `0x7FF`, matching real bus behavior.

---

## ECU State Machine

Each ECU tracks two error counters per ISO 11898-1:

| Counter | Increment | Decrement |
|---------|-----------|-----------|
| TEC (transmit) | +8 per TX error | -1 per successful frame |
| REC (receive)  | +1 per RX error | -1 per successful frame |

State transitions:

```
ACTIVE ──(TEC≥96 or REC≥96)──► WARNING
WARNING ──(TEC≥128 or REC≥128)──► ERROR_PASSIVE
ERROR_PASSIVE ──(TEC≥256)──► BUS_OFF  (silent — no transmit)
BUS_OFF ──(manual reset)──► ACTIVE
```

This matches the exact state machine defined in ISO 11898-1 §6.13.

---

## Fault Injector Design

The `FaultInjector` sits between ECU `send()` calls and `CANBus::transmit()`.
It intercepts frames and applies one of four fault modes:

| Fault | Effect | Real-world analog |
|-------|--------|-------------------|
| `BIT_FLIP` | XOR one random payload bit | EMI / signal noise |
| `CRC_CORRUPT` | XOR the CRC field | Bus contention / glitch |
| `FRAME_DROP` | Discard silently | Babbling node suppression |
| `DUPLICATE` | Transmit twice | Retransmit without ACK |

The injector supports a configurable fault rate (0.0–1.0) and forced single-fault
injection for deterministic testing.

---

## Signal Encoding

All multi-byte values are **big-endian** (Motorola byte order), which is the
standard for CAN signals in automotive DBC files. See `can-protocol.md` for
the full signal table.
