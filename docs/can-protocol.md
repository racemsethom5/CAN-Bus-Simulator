# CAN Protocol Reference — Mini DBC

Signal definitions for the CAN Bus Simulator. All multi-byte values use
**Motorola (big-endian) byte order**, MSB first.

---

## Bus Configuration

| Parameter     | Value         |
|---------------|---------------|
| Standard       | CAN 2.0A      |
| Identifier     | 11-bit        |
| Max payload    | 8 bytes       |
| CRC polynomial | 0x4599 (CRC-15) |
| Byte order     | Motorola / big-endian |

---

## Message Definitions

### Engine ECU Messages

#### 0x0C0 — ENGINE_RPM
| Field       | Value              |
|-------------|--------------------|
| DLC         | 2                  |
| Cycle time  | 20 ms (50 Hz)      |
| Sender      | EngineECU          |

| Signal | Start byte | Length | Factor | Offset | Range      | Unit |
|--------|-----------|--------|--------|--------|------------|------|
| RPM    | 0         | 16 bit | 1      | 0      | 0 – 8000   | rpm  |

**Encoding:** `data[0]` = high byte, `data[1]` = low byte  
**Example:** 3000 rpm → `0x0B 0xB8`

---

#### 0x0C1 — ENGINE_THROTTLE
| Field       | Value          |
|-------------|----------------|
| DLC         | 1              |
| Cycle time  | 20 ms (50 Hz)  |
| Sender      | EngineECU      |

| Signal   | Start byte | Length | Factor | Offset | Range    | Unit |
|----------|-----------|--------|--------|--------|----------|------|
| Throttle | 0         | 8 bit  | 1      | 0      | 0 – 100  | %    |

---

#### 0x0C2 — ENGINE_COOLANT
| Field       | Value          |
|-------------|----------------|
| DLC         | 1              |
| Cycle time  | 20 ms (50 Hz)  |
| Sender      | EngineECU      |

| Signal       | Start byte | Length | Factor | Offset | Range     | Unit |
|--------------|-----------|--------|--------|--------|-----------|------|
| Coolant_Temp | 0         | 8 bit  | 1      | 0      | 0 – 150   | °C   |

---

### ABS ECU Messages

#### 0x1A0 — ABS_WHEEL_SPEED
| Field       | Value           |
|-------------|-----------------|
| DLC         | 4               |
| Cycle time  | 10 ms (100 Hz)  |
| Sender      | ABSECU          |

| Signal      | Byte | Length | Range    | Unit  |
|-------------|------|--------|----------|-------|
| Speed_FL    | 0    | 8 bit  | 0 – 255  | km/h  |
| Speed_FR    | 1    | 8 bit  | 0 – 255  | km/h  |
| Speed_RL    | 2    | 8 bit  | 0 – 255  | km/h  |
| Speed_RR    | 3    | 8 bit  | 0 – 255  | km/h  |

**Vehicle speed** = average of all four wheels.

---

#### 0x1A1 — ABS_BRAKE_STATUS
| Field       | Value           |
|-------------|-----------------|
| DLC         | 1               |
| Cycle time  | 10 ms (100 Hz)  |
| Sender      | ABSECU          |

| Signal      | Bit | Meaning          |
|-------------|-----|------------------|
| Brake_FL    | 0   | 1 = braking      |
| Brake_FR    | 1   | 1 = braking      |
| Brake_RL    | 2   | 1 = braking      |
| Brake_RR    | 3   | 1 = braking      |
| Reserved    | 4–7 | always 0         |

**Example:** All four wheels braking → `0x0F`

---

#### 0x1A2 — ABS_ACTIVE
| Field       | Value           |
|-------------|-----------------|
| DLC         | 1               |
| Cycle time  | 10 ms (100 Hz)  |
| Sender      | ABSECU          |

| Signal     | Byte | Values          |
|------------|------|-----------------|
| ABS_Active | 0    | 0 = off, 1 = on |

---

## ECU Error Counters

| Threshold | State          |
|-----------|----------------|
| TEC or REC ≥ 96  | WARNING        |
| TEC or REC ≥ 128 | ERROR_PASSIVE  |
| TEC ≥ 256         | BUS_OFF        |

---

## Receiver

**DashboardECU** subscribes to all messages above and decodes them using
`SignalParser`. It does not transmit any frames.
