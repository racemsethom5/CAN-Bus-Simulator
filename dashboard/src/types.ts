// Mirrors C++ CANFrame struct exactly
export interface CANFrame {
  id: number;       // 11-bit arbitration ID
  dlc: number;      // 0–8
  data: number[];   // payload bytes
  crc: number;      // CRC-15
  is_remote: boolean;
  is_error: boolean;
  timestamp: number; // ms since simulation start
}

// Mirrors C++ ECUState enum
export type ECUState =
  | 'ACTIVE'
  | 'WARNING'
  | 'ERROR_PASSIVE'
  | 'BUS_OFF';

// Mirrors C++ BusState enum
export type BusState = 'ACTIVE' | 'WARNING' | 'BUS_OFF';

// Decoded engineering values
export interface SignalSnapshot {
  rpm:         number;   // 0–8000
  speed:       number;   // km/h
  throttle:    number;   // 0–100 %
  coolant:     number;   // °C
  brakeMask:   number;   // bitmask FL/FR/RL/RR
  absActive:   boolean;
}

// Fault event shown in the fault log
export interface FaultEvent {
  timestamp: number;
  type: 'BIT_FLIP' | 'CRC_CORRUPT' | 'FRAME_DROP' | 'DUPLICATE';
  frameId: number;
}

// CAN IDs — mirrors C++ namespace constants
export const EngineID = {
  RPM:      0x0C0,
  THROTTLE: 0x0C1,
  COOLANT:  0x0C2,
} as const;

export const ABSID = {
  WHEEL_SPEED:  0x1A0,
  BRAKE_STATUS: 0x1A1,
  ABS_ACTIVE:   0x1A2,
} as const;
