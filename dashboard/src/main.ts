import { BusSimulation } from './simulation';
import { CANFrame, FaultEvent, EngineID, ABSID } from './types';

const sim = new BusSimulation();

// ── DOM refs ──────────────────────────────────────────────────────────────
const rpmNeedle    = document.getElementById('rpm-needle')    as unknown as SVGLineElement;
const rpmValue     = document.getElementById('rpm-value')     as HTMLElement;
const speedFill    = document.getElementById('speed-fill')    as HTMLElement;
const speedValue   = document.getElementById('speed-value')   as HTMLElement;
const throttleFill = document.getElementById('throttle-fill') as HTMLElement;
const throttleValue= document.getElementById('throttle-value')as HTMLElement;
const coolantFill  = document.getElementById('coolant-fill')  as HTMLElement;
const coolantValue = document.getElementById('coolant-value') as HTMLElement;
const absIndicator = document.getElementById('abs-indicator') as HTMLElement;
const busState     = document.getElementById('bus-state')     as HTMLElement;
const phaseLabel   = document.getElementById('phase-label')   as HTMLElement;
const msgFeed      = document.getElementById('msg-feed')      as HTMLElement;
const faultLog     = document.getElementById('fault-log')     as HTMLElement;
const brakeWheels  = ['FL','FR','RL','RR'].map(w =>
  document.getElementById(`brake-${w}`) as HTMLElement
);

// ── RPM needle: 270° arc from -135° to +135° ─────────────────────────────
function setRPM(rpm: number) {
  const angle = -135 + (Math.min(rpm, 8000) / 8000) * 270;
  const rad   = (angle * Math.PI) / 180;
  const cx = 100, cy = 100, r = 75;
  const x2 = cx + r * Math.sin(rad);
  const y2 = cy - r * Math.cos(rad);
  rpmNeedle.setAttribute('x2', x2.toFixed(1));
  rpmNeedle.setAttribute('y2', y2.toFixed(1));
  rpmValue.textContent = Math.round(rpm).toLocaleString();

  // Red zone above 6000
  rpmNeedle.setAttribute('stroke', rpm > 6000 ? '#ef4444' : '#22c55e');
}

function setBar(fill: HTMLElement, label: HTMLElement, value: number,
                max: number, unit: string) {
  fill.style.width = `${Math.min(100, (value / max) * 100)}%`;
  label.textContent = `${Math.round(value)} ${unit}`;
}

// ── Frame → UI ─────────────────────────────────────────────────────────────
function onFrame(frame: CANFrame) {
  const s = sim.signals;

  switch (frame.id) {
    case EngineID.RPM:
      setRPM(s.rpm);
      break;
    case EngineID.THROTTLE:
      setBar(throttleFill, throttleValue, s.throttle, 100, '%');
      break;
    case EngineID.COOLANT:
      setBar(coolantFill, coolantValue, s.coolant, 150, '°C');
      coolantFill.style.background =
        s.coolant > 110 ? '#ef4444' : s.coolant > 100 ? '#f97316' : '#22c55e';
      break;
    case ABSID.WHEEL_SPEED:
      setBar(speedFill, speedValue, s.speed, 240, 'km/h');
      break;
    case ABSID.BRAKE_STATUS:
      brakeWheels.forEach((el, i) => {
        el.classList.toggle('active', Boolean((s.brakeMask >> i) & 1));
      });
      break;
    case ABSID.ABS_ACTIVE:
      absIndicator.classList.toggle('active', s.absActive);
      absIndicator.textContent = s.absActive ? 'ABS ON' : 'ABS';
      break;
  }

  // Phase label
  phaseLabel.textContent = (sim as unknown as { phase: string }).phase ?? '';

  // Live message feed — keep last 80 entries
  appendMsg(frame);
}

function appendMsg(frame: CANFrame) {
  const hex  = (n: number, w: number) => n.toString(16).toUpperCase().padStart(w, '0');
  const data = frame.data.slice(0, frame.dlc).map(b => hex(b, 2)).join(' ');
  const row  = document.createElement('div');
  row.className = 'msg-row' + (frame.is_error ? ' error' : '');
  row.innerHTML =
    `<span class="msg-ts">${(frame.timestamp / 1000).toFixed(3)}s</span>` +
    `<span class="msg-id">0x${hex(frame.id, 3)}</span>` +
    `<span class="msg-dlc">[${frame.dlc}]</span>` +
    `<span class="msg-data">${data}</span>` +
    (frame.is_error ? `<span class="msg-err">ERR</span>` : '');

  msgFeed.prepend(row);
  if (msgFeed.children.length > 80)
    msgFeed.removeChild(msgFeed.lastChild!);
}

function onFault(fault: FaultEvent) {
  const hex = (n: number) => '0x' + n.toString(16).toUpperCase().padStart(3, '0');
  const row = document.createElement('div');
  row.className = 'fault-row';
  row.innerHTML =
    `<span class="fault-ts">${(fault.timestamp / 1000).toFixed(3)}s</span>` +
    `<span class="fault-type">${fault.type}</span>` +
    `<span class="fault-id">ID=${hex(fault.frameId)}</span>`;

  faultLog.prepend(row);
  if (faultLog.children.length > 20)
    faultLog.removeChild(faultLog.lastChild!);

  // Flash bus state
  busState.textContent  = 'FAULT';
  busState.className    = 'state-badge warning';
  setTimeout(() => {
    busState.textContent = 'ACTIVE';
    busState.className   = 'state-badge active';
  }, 500);
}

sim.onFrame(onFrame);
sim.onFault(onFault);
sim.start(50);
