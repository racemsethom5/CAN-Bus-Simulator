import { CANFrame, FaultEvent, SignalSnapshot, EngineID, ABSID } from './types';

export type FrameCallback = (frame: CANFrame) => void;
export type FaultCallback = (fault: FaultEvent) => void;

// CRC-15 — same polynomial as C++ implementation (0x4599)
function computeCRC(id: number, dlc: number, data: number[]): number {
  const POLY = 0x4599;
  let crc = 0;

  const processBit = (bit: number) => {
    const msb = (crc >> 14) & 1;
    crc = (crc << 1) & 0x7FFF;
    if (msb ^ bit) crc ^= POLY;
  };

  const processByte = (byte: number, bits = 8) => {
    for (let i = bits - 1; i >= 0; i--) processBit((byte >> i) & 1);
  };

  processByte(id >> 3, 8);
  processByte(id & 0x7, 3);
  processBit(0);           // RTR
  processByte(dlc & 0xF, 4);
  data.slice(0, dlc).forEach(b => processByte(b));

  return crc & 0x7FFF;
}

function makeFrame(
  id: number, dlc: number, data: number[],
  startTime: number, error = false
): CANFrame {
  const crc = computeCRC(id, dlc, data);
  return {
    id, dlc, data, crc,
    is_remote: false,
    is_error: error,
    timestamp: Date.now() - startTime,
  };
}

// Simulation state machine
type Phase = 'IDLE' | 'ACCEL' | 'CRUISE' | 'BRAKE' | 'ABS';

export class BusSimulation {
  private frameCallbacks: FrameCallback[] = [];
  private faultCallbacks: FaultCallback[] = [];
  private timer: ReturnType<typeof setInterval> | null = null;
  private startTime = Date.now();
  private phase: Phase = 'IDLE';
  private phaseTimer = 0;

  // Live signal state
  signals: SignalSnapshot = {
    rpm: 800, speed: 0, throttle: 5,
    coolant: 90, brakeMask: 0, absActive: false,
  };

  onFrame(cb: FrameCallback)  { this.frameCallbacks.push(cb); }
  onFault(cb: FaultCallback)  { this.faultCallbacks.push(cb); }

  start(intervalMs = 50) {
    this.startTime = Date.now();
    this.timer = setInterval(() => this.tick(), intervalMs);
  }

  stop() {
    if (this.timer !== null) clearInterval(this.timer);
  }

  private emit(frame: CANFrame) {
    this.frameCallbacks.forEach(cb => cb(frame));
  }

  private emitFault(fault: FaultEvent) {
    this.faultCallbacks.forEach(cb => cb(fault));
  }

  private tick() {
    this.phaseTimer++;
    this.advancePhase();
    this.emitSignals();

    // Random fault injection (~5% chance per tick)
    if (Math.random() < 0.05) this.injectFault();
  }

  private advancePhase() {
    const s = this.signals;

    switch (this.phase) {
      case 'IDLE':
        s.rpm = 800 + Math.random() * 50;
        s.throttle = 5;
        s.speed = 0;
        if (this.phaseTimer > 20) this.switchPhase('ACCEL');
        break;

      case 'ACCEL':
        s.rpm    = Math.min(6500, s.rpm + 120);
        s.throttle = Math.min(100, s.throttle + 3);
        s.speed  = Math.min(180, s.speed + 2.5);
        s.brakeMask = 0;
        s.absActive = false;
        if (s.rpm >= 6500) this.switchPhase('CRUISE');
        break;

      case 'CRUISE':
        s.rpm = 4500 + Math.sin(this.phaseTimer * 0.2) * 200;
        s.throttle = 45 + Math.sin(this.phaseTimer * 0.3) * 5;
        if (this.phaseTimer > 30) this.switchPhase('BRAKE');
        break;

      case 'BRAKE':
        s.throttle = 0;
        s.rpm    = Math.max(800, s.rpm - 200);
        s.speed  = Math.max(0, s.speed - 6);
        s.brakeMask = 0x0F;
        if (s.speed < 80 && !s.absActive) this.switchPhase('ABS');
        if (s.speed <= 0) this.switchPhase('IDLE');
        break;

      case 'ABS':
        s.absActive = true;
        s.speed  = Math.max(0, s.speed - 3);
        s.rpm    = Math.max(800, s.rpm - 100);
        // ABS pulsing — brakeMask alternates
        s.brakeMask = this.phaseTimer % 2 === 0 ? 0x0F : 0x05;
        if (s.speed <= 0) this.switchPhase('IDLE');
        break;
    }

    // Coolant slowly rises under load, cools at idle
    if (s.rpm > 4000) s.coolant = Math.min(120, s.coolant + 0.1);
    else              s.coolant = Math.max(80,  s.coolant - 0.05);
  }

  private switchPhase(next: Phase) {
    this.phase = next;
    this.phaseTimer = 0;
  }

  private emitSignals() {
    const s = this.signals;
    const t = this.startTime;

    const rpmHi  = (Math.round(s.rpm) >> 8) & 0xFF;
    const rpmLo  = Math.round(s.rpm) & 0xFF;

    this.emit(makeFrame(EngineID.RPM,      2, [rpmHi, rpmLo], t));
    this.emit(makeFrame(EngineID.THROTTLE, 1, [Math.round(s.throttle)], t));
    this.emit(makeFrame(EngineID.COOLANT,  1, [Math.round(s.coolant)], t));

    const ws = Math.round(s.speed);
    this.emit(makeFrame(ABSID.WHEEL_SPEED,  4, [ws, ws, ws, ws], t));
    this.emit(makeFrame(ABSID.BRAKE_STATUS, 1, [s.brakeMask], t));
    this.emit(makeFrame(ABSID.ABS_ACTIVE,   1, [s.absActive ? 1 : 0], t));
  }

  private injectFault() {
    const types = ['BIT_FLIP', 'CRC_CORRUPT', 'FRAME_DROP', 'DUPLICATE'] as const;
    const type  = types[Math.floor(Math.random() * types.length)];
    const ids   = [EngineID.RPM, EngineID.THROTTLE, ABSID.WHEEL_SPEED];
    const frameId = ids[Math.floor(Math.random() * ids.length)];

    this.emitFault({ timestamp: Date.now() - this.startTime, type, frameId });

    if (type !== 'FRAME_DROP') {
      const bad = makeFrame(frameId, 2, [0xFF, 0xFF], this.startTime, true);
      bad.crc ^= 0x1234;
      this.emit(bad);
    }
  }
}
