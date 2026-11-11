(() => {
'use strict';
const $ = s => document.querySelector(s);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const lerp = (a, b, t) => a + (b - a) * t;
const gauss = (x, m, s) => Math.exp(-(((x - m) / s) ** 2));
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

/* ---------- Certified data (EASA TCDS E.073 Issue 05, Arriel 1D1) ---------- */
const C = {
  N1_RPM: 51800, N2_RPM: 41420, OUT_RPM: 5976, P_TO: 531, P_MCP: 466,
  TOT_TO: 845, TOT_MCP: 795, TOT_START: 865, TOT_TRANS: 870,
  N1_TO: 101.9, N1_MCP: 98, N1_T20: 103.1, N1_T5: 107.5, N1_MIN: 67,
  N2_MAXST: 108.5, N2_MAXTR: 120.5, N2_MINTR: 86,
  Q_MAX: 83, Q_20: 91.5, OILT_MAX: 115, LHV: 43136
};
const RATIO = C.N2_RPM / C.OUT_RPM;
/* Model constants */
const N1_STOP = 104.5, N1_IDLE = 68, CUTOUT = 45, IGN_ON = 10, LIGHTOFF = 12;

/* ---------- State ---------- */
const S = {};
const E = [false, false, false, false, false, false];
function amb() {
  const TK = S.oat + 273.15, th = TK / 288.15, de = Math.pow(1 - 6.8756e-6 * S.alt, 5.2559);
  return { TK, th, de, sq: Math.sqrt(th) };
}
const preq = () => 140 + S.coll * 4.6;
const fr = n1c => clamp((n1c - 60) / 41.9, 0, 1.25);
const kDeg = () => (S.foul ? 0.9 : 1) * (S.bleed ? 0.94 : 1);
function pGG(N1, a) {
  return (C.P_TO * Math.pow(fr(N1 / a.sq), 1.5) + 0.35 * Math.max(0, N1 - 20)) * a.de / a.sq * kDeg();
}
function n1For(P, a) {
  let n = 85;
  for (let i = 0; i < 25; i++) {
    const p = pGG(n, a), dp = (pGG(n + 0.1, a) - p) / 0.1;
    n = clamp(n - (p - P) / Math.max(dp, 0.5), 20, 115);
  }
  return n;
}
function t45C(N1, a) {
  const f = fr(N1 / a.sq);
  return a.th * (773 + 345 * Math.pow(f, 1.6)) + (S.foul ? 35 : 0) + (S.bleed ? 22 : 0) - 273.15;
}

function resetState() {
  Object.assign(S, {
    t: 0, N1: 0, N2: 100, NR: 100, TOT: 0, oilT: 78, starter: false, startHeld: false, cutout: false, ign: false, comb: true, starting: false,
    lever: 'flight', coll: 45, airborne: true, oat: 15, alt: 0, hot: false, foul: false, bleed: false,
    speed: 1, dN1: 0, P: 0, q: 0, oilP: 0, coupled: true, startPeak: 0, lastStartHot: false, flameoutAt: null,
    sawStop: false, startFromStop: false, n1Over: 0, qOver: 0
  });
  const a = amb();
  S.N1 = n1For(preq(), a);
  S.TOT = t45C(S.N1, a);
  for (const k in alarmState) alarmState[k] = null;
  syncInputs();
}

/* ---------- Physics ---------- */
function step(dt) {
  const a = amb();
  S.t += dt;
  const fuel = S.lever !== 'off';
  // Momentary starter: powered only while the switch is held. The cut-out relay drops it at 45% N1
  // and stays open until the switch is released.
  if (!S.startHeld) S.cutout = false;
  else if (S.N1 >= CUTOUT) S.cutout = true;
  S.starter = S.startHeld && !S.cutout;
  if (S.comb && !fuel) {
    if (S.starting && S.lastStartHot && S.startPeak < C.TOT_START) E[4] = true;
    S.comb = false; S.starting = false;
  }
  S.ign = S.starter && fuel && !S.comb && S.N1 >= IGN_ON;
  if (S.ign && S.N1 >= LIGHTOFF) {
    S.comb = true; S.starting = true; S.startPeak = S.TOT; S.lastStartHot = S.hot;
    S.flameoutAt = null; S.startFromStop = S.sawStop; S.sawStop = false;
  }

  let dN1;
  if (S.comb && S.starting) {
    // Without the starter, the turbine only accelerates the gas generator on its own above ~25% N1.
    const self = S.starter ? 2.4 : 1.9 * clamp((S.N1 - 25) / 15, -0.4, 1);
    dN1 = self * (S.hot && S.lastStartHot ? 0.6 : 1);
    if (S.N1 >= 67.5) {
      S.starting = false;
      if (S.startFromStop && !S.lastStartHot) E[2] = true;
    }
  } else if (S.comb) {
    let cmd = S.lever === 'idle' ? N1_IDLE : n1For(preq(), a) + 2.5 * (100 - S.N2);
    cmd = clamp(cmd, N1_IDLE, N1_STOP);
    dN1 = clamp((cmd - S.N1) / 0.9, -7, 5);
  } else if (S.starter) {
    dN1 = (22 - S.N1) * 0.35;
  } else {
    dN1 = -(0.07 * S.N1 + 0.4);
  }
  S.N1 = clamp(S.N1 + dN1 * dt, 0, 115);
  S.dN1 = S.N1 > 0 ? dN1 : 0;
  if (!S.comb && !S.starter && S.N1 < 3) S.sawStop = true;

  const Pgg = S.comb ? pGG(S.N1, a) : 0;
  const pl = preq() * Math.pow(Math.max(S.NR, 0) / 100, 2.5);
  if (!S.comb && S.airborne) {
    const target = clamp(108 - 0.42 * S.coll, 0, 115);
    S.NR += (target - S.NR) * (1 - Math.exp(-dt * 0.35));
    S.N2 = Math.max(0, S.N2 - (0.08 * S.N2 + 0.5) * dt);
    S.coupled = false;
  } else if (S.N2 < S.NR - 0.3) {
    S.N2 = Math.max(0, S.N2 + (25 * Pgg / Math.max(S.N2, 20) - 0.1 * S.N2) * dt);
    S.NR = Math.max(0, S.NR + (-3 * pl / Math.max(S.NR, 15) - 0.15) * dt);
    S.coupled = false;
    if (S.N2 >= S.NR) S.N2 = S.NR;
  } else {
    const dW = 3 * (Pgg - pl) / Math.max(S.NR, 15) - (S.comb ? 0 : 0.15);
    S.NR = clamp(S.NR + dW * dt, 0, 130);
    S.N2 = S.NR; S.coupled = true;
  }
  S.P = S.coupled ? Pgg : 0;
  const w = C.OUT_RPM * (S.N2 / 100) * 2 * Math.PI / 60;
  S.q = S.P > 0 && S.N2 > 3 ? Math.min(S.P * 1000 / w / 10, 140) : 0;

  let tT, tau;
  if (S.comb) {
    tT = t45C(S.N1, a) + 9 * Math.max(0, S.dN1);
    if (S.starting) tT += 270 * gauss(S.N1, 32, 13) + (S.lastStartHot ? 380 * gauss(S.N1, 27, 9) : 0);
    tau = S.starting ? 1.2 : 1.5;
  } else {
    tT = S.oat; tau = S.starter ? 2.5 : 7;
  }
  S.TOT += (tT - S.TOT) * (1 - Math.exp(-dt / tau));
  if (S.starting) S.startPeak = Math.max(S.startPeak, S.TOT);

  const oilTarget = S.N1 > 5 ? S.oat + 30 + 50 * (Pgg / C.P_TO) + 10 * (S.N1 / 100) : S.oat;
  S.oilT += (oilTarget - S.oilT) * (1 - Math.exp(-dt / 45));
  S.oilP = S.N1 > 3 ? (150 + 3900 * Math.pow(S.N1 / 100, 1.4)) * (1 + clamp((50 - S.oilT) / 160, 0, 0.45)) : 0;

  S.n1Over = S.N1 > C.N1_T20 ? S.n1Over + dt : 0;
  S.qOver = S.q > C.Q_MAX ? S.qOver + dt : 0;
  checkAlarms();
  checkExperiments();
}

/* ---------- Thermodynamic stations (teaching approximations) ---------- */
function thermoCalc(N1, TOTc, TK, de, comb, bleed) {
  const sq = Math.sqrt(TK / 288.15);
  const x = clamp(N1 / sq / 100, 0, 1.1);
  const P0 = 1.01325 * de, T0 = TK;
  const PR = 1 + 7 * x * x, PRa = Math.pow(PR, 0.32), PRc = PR / PRa;
  const P1 = P0 * (1 - 0.01 * x * x), T1 = T0;
  const T15 = T1 * (1 + (Math.pow(PRa, 0.2857) - 1) / 0.85), P15 = P1 * PRa;
  const T2 = T15 * (1 + (Math.pow(PRc, 0.2857) - 1) / 0.78), P2 = P15 * PRc;
  const mdot = 2.5 * Math.pow(x, 1.8) * de / sq * (bleed ? 0.96 : 1);
  const T45 = TOTc + 273.15;
  const P5 = P0 * (1 + 0.03 * x * x);
  let T4, P4, P45, T5;
  if (comb) {
    T4 = T45 + (1.005 / 1.15) * (T2 - T1) / 0.98;
    P4 = P2 * 0.95;
    const r = clamp(1 - (T4 - T45) / (0.85 * T4), 0.05, 1);
    P45 = Math.max(P4 * Math.pow(r, 1 / 0.248), P5 * 1.002);
    T5 = T45 * (1 - 0.85 * (1 - Math.pow(P5 / P45, 0.248)));
  } else {
    T4 = T45; P4 = P2 * 0.97; P45 = Math.max(P5, P4 * 0.8); T5 = T45;
  }
  const Wc = mdot * 1.005 * (T2 - T1);
  const Q = comb ? mdot * 1.15 * Math.max(T4 - T2, 0) / 0.98 : 0;
  return {
    PR, mdot, Wc, Q, Wf: Q * 3600 / C.LHV,
    st: [
      { k: '0', name: 'Ambient', P: P0, T: T0 },
      { k: '1', name: 'Compressor inlet', P: P1, T: T1 },
      { k: '1.5', name: 'Axial stage exit', P: P15, T: T15 },
      { k: '2', name: 'Compressor delivery (P2)', P: P2, T: T2 },
      { k: '4', name: 'Gas generator turbine inlet', P: P4, T: T4 },
      { k: '4.5', name: 'Free turbine inlet (TOT)', P: P45, T: T45 },
      { k: '5', name: 'Exhaust', P: P5, T: T5 }
    ]
  };
}
let TH = null;
const REF = (() => { const tmp = thermoCalc(C.N1_TO, C.TOT_TO, 288.15, 1, true, false); return tmp; })();

/* ---------- Alarms and log ---------- */
const LAMPS = [
  ['STARTER', 'info'], ['IGNITION', 'info'], ['GOV', 'info'], ['TOT', ''], ['N1', ''], ['TORQUE', ''],
  ['LOW ROTOR', ''], ['OIL PRESS', ''], ['OIL TEMP', ''], ['ENGINE OUT', '']
];
const alarmState = {};
const logItems = [];
function lampLevels() {
  const L = {};
  L['STARTER'] = S.starter ? 'info' : '';
  L['IGNITION'] = S.ign || (S.starting && S.starter) ? 'info' : '';
  L['GOV'] = S.comb && !S.starting && S.lever === 'flight' ? 'info' : '';
  L['TOT'] = S.TOT > (S.starting ? C.TOT_START : C.TOT_TRANS) ? 'warn' : (S.TOT > C.TOT_TO ? 'caution' : '');
  L['N1'] = (S.N1 > C.N1_T5 || S.n1Over > 20) ? 'warn' : (S.N1 > C.N1_TO ? 'caution' : '');
  L['TORQUE'] = (S.q > C.Q_20 || S.qOver > 20) ? 'warn' : (S.q > C.Q_MAX ? 'caution' : '');
  const lowRotorRelevant = S.airborne || (S.comb && !S.starting && S.lever === 'flight');
  L['LOW ROTOR'] = lowRotorRelevant && S.NR < 95 ? (S.NR < C.N2_MINTR ? 'warn' : 'caution') : (S.N2 > C.N2_MAXST ? 'caution' : '');
  const minP = S.N1 >= 85 ? 1800 : 1300;
  L['OIL PRESS'] = S.N1 >= 70 && S.oilP < minP ? 'warn' : (S.oilP > 5000 ? 'caution' : '');
  L['OIL TEMP'] = S.oilT > C.OILT_MAX ? 'warn' : '';
  L['ENGINE OUT'] = !S.comb && S.airborne ? 'warn' : '';
  return L;
}
const LOGDEF = [
  { id: 'totS', on: () => S.starting && S.TOT > C.TOT_START, sev: 'warn', text: 'Start over-temperature (865 °C)', v: () => S.TOT, u: ' °C', f: 0 },
  { id: 'totW', on: () => !S.starting && S.TOT > C.TOT_TRANS, sev: 'warn', text: 'TOT above transient limit (870 °C)', v: () => S.TOT, u: ' °C', f: 0 },
  { id: 'totC', on: () => !S.starting && S.TOT > C.TOT_TO, sev: 'caution', text: 'TOT above take-off limit (845 °C)', v: () => S.TOT, u: ' °C', f: 0 },
  { id: 'n1C', on: () => S.N1 > C.N1_TO, sev: 'caution', text: 'N1 above take-off limit (101.9%)', v: () => S.N1, u: '%', f: 1 },
  { id: 'qC', on: () => S.q > C.Q_MAX, sev: 'caution', text: 'Torque above 83 m·daN', v: () => S.q, u: ' m·daN', f: 1 },
  { id: 'nrL', on: () => (S.airborne || (S.comb && !S.starting && S.lever === 'flight')) && S.NR < 95, sev: 'caution', text: 'Low rotor speed (below 95%)', v: () => -S.NR, u: '%', f: 1, neg: true },
  { id: 'n2H', on: () => S.N2 > C.N2_MAXST, sev: 'warn', text: 'N2 above 108.5%', v: () => S.N2, u: '%', f: 1 },
  { id: 'opL', on: () => S.N1 >= 70 && S.oilP < (S.N1 >= 85 ? 1800 : 1300), sev: 'warn', text: 'Oil pressure below minimum', v: () => -S.oilP, u: ' hPa', f: 0, neg: true },
  { id: 'otH', on: () => S.oilT > C.OILT_MAX, sev: 'warn', text: 'Oil temperature above 115 °C', v: () => S.oilT, u: ' °C', f: 0 }
];
function checkAlarms() {
  for (const d of LOGDEF) {
    const on = d.on();
    if (on && !alarmState[d.id]) {
      const item = { t: S.t, d, peak: d.v() };
      alarmState[d.id] = item; logItems.unshift(item);
      if (logItems.length > 12) logItems.pop();
      logDirty = true;
    } else if (on) {
      const it = alarmState[d.id]; const v = d.v();
      if (v > it.peak) { it.peak = v; logDirty = true; }
    } else if (!on && alarmState[d.id]) alarmState[d.id] = null;
  }
}
let logDirty = true;
function renderLog() {
  const ul = $('#log');
  if (!logItems.length) { ul.innerHTML = '<li class="empty">No exceedances yet.</li>'; return; }
  ul.innerHTML = logItems.map(it => {
    const v = it.d.neg ? -it.peak : it.peak;
    const label = it.d.neg ? 'lowest' : 'peak';
    return `<li class="${it.d.sev}"><time>${fmtT(it.t)}</time><span><b>${it.d.text}</b> · ${label} ${v.toFixed(it.d.f)}${it.d.u}</span></li>`;
  }).join('');
}
const fmtT = t => { const m = Math.floor(t / 60), s = Math.floor(t % 60); return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`; };

let lastNote = '', noteUntil = 0;
function note(s) { lastNote = s; noteUntil = S.t + 3; }

/* ---------- Experiments ---------- */
const EXP = [
  'In FLIGHT, raise the collective to 70%. N1 climbs to supply the power while N2 stays near 100%.',
  'Set OAT to 35 °C or more and altitude to 7,000 ft or more. Raise the collective until TOT reaches about 845 °C, then compare shaft power with the 531 kW sea-level rating.',
  'Shut down: On ground, collective down, lever to OFF. When N1 is below 3%, set the lever to IDLE, then press and hold the starter. Release it after the 45% N1 cut-out and let the engine stabilize at idle.',
  'From idle, move the lever to FLIGHT and let the governor bring N2 to 100%.',
  'Tick Hot start, shut down, then start again. Cut the fuel (lever OFF) before TOT passes 865 °C.',
  'Airborne, press Flameout now, then lower the collective to 25% or less to keep rotor speed at 90% or more.'
];
function checkExperiments() {
  if (!E[0] && S.comb && S.lever === 'flight' && S.coll >= 70 && Math.abs(S.N2 - 100) < 3 && S.N1 > 96) E[0] = true;
  if (!E[1] && S.comb && S.oat >= 35 && S.alt >= 7000 && S.TOT >= 835) E[1] = true;
  if (!E[3] && E[2] && S.comb && !S.starting && S.lever === 'flight' && Math.abs(S.N2 - 100) < 1.5) E[3] = true;
  if (!E[5] && S.flameoutAt !== null && S.airborne && !S.comb && S.coll <= 25 && S.NR >= 90 && S.t - S.flameoutAt > 4) E[5] = true;
}
function renderExp() {
  $('#exp').innerHTML = EXP.map((t, i) => `<li class="${E[i] ? 'done' : ''}">${t}</li>`).join('');
}
let expSig = '';

/* ---------- Parts content ---------- */
const PARTS = [
  { id: 'intake', name: 'Air intake', mod: 'Front casing',
    body: '<p>Air enters through the axial intake at the front and flows around the accessory gearbox housing to reach the compressor face. Everything downstream depends on how much air the engine can swallow each second.</p><p>Hot or high-altitude air is thinner, so the same N1 moves less mass and makes less power.</p>',
    facts: [['Air mass flow at take-off', '≈ 2.5 kg/s', 'model'], ['Rating conditions', '15 °C, 1013 hPa, no bleed', 'TCDS']],
    tryit: 'Raise OAT to 40 °C and watch shaft power and TOT at the same collective.' },
  { id: 'axial', name: 'Axial compressor', mod: 'Module M02',
    body: '<p>One axial stage: a ring of rotating blades followed by a ring of stator vanes. It gives the air its first pressure rise and straightens the flow before the centrifugal stage.</p><p>It turns with the gas generator at N1.</p>',
    facts: [['Stages', '1 axial', 'TCDS'], ['Pressure rise at take-off', '≈ 1.9 : 1', 'model'], ['N1 at 100%', '51,800 rpm', 'TCDS']],
    tryit: 'Tick Compressor fouling. Dirty blades move less air, so power drops and TOT rises.' },
  { id: 'centrifugal', name: 'Centrifugal compressor', mod: 'Module M03 · gas generator',
    body: '<p>The impeller flings the air outward at high speed. The diffuser around its tip slows the air down and turns that speed into pressure.</p><p>Both stages together reach an overall pressure ratio of about 8 : 1, and compression alone heats the air to around 300 °C at take-off. Cabin heating air is bled off here (P2 bleed).</p>',
    facts: [['Overall pressure ratio', '≈ 8 : 1', 'approx.'], ['Max P2 bleed at take-off', '100 g/s', 'TCDS']],
    tryit: 'Tick P2 bleed open. The engine burns more fuel and runs hotter for the same shaft power.' },
  { id: 'combustor', name: 'Combustion chamber', mod: 'Module M03 · gas generator',
    body: '<p>An annular chamber surrounds the shaft. Fuel is fed through the hollow gas generator shaft and thrown outward by a spinning injection wheel, so the spray gets finer as N1 rises. Igniter plugs and start injectors light the chamber during a start.</p><p>Only part of the air burns. The rest flows around the liner and enters through rows of holes to cool the walls and dilute the gas before it reaches the turbine.</p>',
    facts: [['Turbine inlet gas', '≈ 1,100 °C at take-off', 'model'], ['Fuel heating value for ratings', '43,136 kJ/kg', 'TCDS']],
    tryit: 'Raise the collective and watch the flame and fuel spray grow.' },
  { id: 'ggturbine', name: 'Gas generator turbine', mod: 'Module M03 · gas generator',
    body: '<p>Two turbine stages on the same shaft as the compressors. Their only job is to keep the gas generator running: they drive both compressors and, through the accessory gearbox, the fuel control, oil pumps and starter-generator.</p><p>At take-off the compressors absorb more power than the engine sends to the rotor. Check the turbine work split.</p>',
    facts: [['Stages', '2', 'TCDS'], ['N1 max continuous', '98%', 'TCDS'], ['N1 take-off', '101.9%', 'TCDS'], ['N1 transient', '103.1% 20 s · 107.5% 5 s', 'TCDS'], ['Minimum N1', '67%', 'TCDS']],
    tryit: 'Pull the collective to 90%. N1 runs into the fuel stop and the N1 lamp lights.' },
  { id: 'tot', name: 'TOT probes (t4.5)', mod: 'Between the turbines',
    body: '<p>Thermocouples sit in the gas between the gas generator turbine and the free turbine. Turbine inlet gas is too hot to measure reliably, but it tracks this reading closely, so TOT is the temperature the pilot watches.</p><p>The fuel control governs speeds, not temperature. Keeping TOT inside its limits is the pilot\'s job.</p>',
    facts: [['Max continuous', '795 °C', 'TCDS'], ['Take-off', '845 °C', 'TCDS'], ['Starting', '865 °C', 'TCDS'], ['Transient (<20 s, VEMD)', '870 °C', 'TCDS']],
    tryit: 'Slam the collective from 20% to 80%. TOT overshoots while N1 accelerates.' },
  { id: 'pt', name: 'Free turbine', mod: 'Module M04',
    body: '<p>A single stage on its own shaft, with no mechanical link to the gas generator. Only the gas couples them. This is the key idea of a free-turbine turboshaft: the rotor and free turbine stay at constant speed while the gas generator speeds up or slows down to change the power.</p><p>It also means the starter only turns the light gas generator, not the rotor.</p>',
    facts: [['Stages', '1', 'TCDS'], ['N2 at 100%', '41,420 rpm', 'TCDS'], ['Max stabilized', '108.5%', 'TCDS'], ['5 s transient', '120.5% / 86%', 'TCDS']],
    tryit: 'Move the collective. N1 changes a lot, N2 barely moves.' },
  { id: 'exhaust', name: 'Exhaust', mod: 'Module M04',
    body: '<p>Gas leaves the free turbine at roughly 600 °C and is turned out through the exhaust duct. A turboshaft takes as much energy as it can in its turbines, so the exhaust gives almost no useful thrust.</p><p>The heat it carries away is the biggest loss in the energy panel.</p>',
    facts: [['Exhaust temperature at take-off', '≈ 630 °C', 'model'], ['Thermal efficiency at take-off', '≈ 23%', 'model']],
    tryit: 'Compare the energy bar at idle and at high collective.' },
  { id: 'rgb', name: 'Reduction gearbox', mod: 'Module M05 · rear',
    body: '<p>At the rear, helical gears reduce free-turbine speed from 41,420 rpm to 5,976 rpm and multiply torque by the same factor.</p><p>The output gear drives the transmission shaft, which runs forward under the engine to the front flange.</p>',
    facts: [['Output shaft speed', '5,976 rpm', 'TCDS'], ['Reduction ratio', '6.93 : 1', 'derived'], ['Max torque', '83 m·daN', 'TCDS'], ['20 s torque', '91.5 m·daN', 'TCDS']],
    tryit: 'Watch the gear inset and the two torque values as you raise the collective.' },
  { id: 'agb', name: 'Transmission shaft and accessory gearbox', mod: 'Module M01 · front',
    body: '<p>The power output shaft runs forward from the reduction gearbox inside a protective tube to the front flange, which connects to the helicopter\'s main gearbox. A free wheel in the drive line lets the rotor keep turning if the engine stops. That is what makes autorotation possible.</p><p>The accessory gearbox at the front is driven by the gas generator. It runs the fuel control, oil pumps and the starter-generator.</p>',
    facts: [['Length × height × width', '1.20 × 0.61 × 0.47 m', 'TCDS'], ['Dry weight', '122 kg', 'TCDS'], ['Single-engine special condition', 'free wheel', 'TCDS']],
    tryit: 'Airborne, press Flameout now. N2 and rotor speed split apart: the free wheel at work.' },
  { id: 'fcu', name: 'Fuel control unit', mod: 'On the accessory gearbox',
    body: '<p>A hydromechanical unit meters the fuel. It contains a proportional free-turbine speed governor and a proportional-integral gas generator speed control. In FLIGHT it holds N2 near 100% by adjusting fuel, and so N1. At IDLE it holds N1 near its idle setting.</p><p>The sim also feeds the collective position forward to the governor, so N2 sags less when you pull power.</p>',
    facts: [['Type', 'Hydromechanical', 'TCDS'], ['N2 control', 'proportional', 'TCDS'], ['N1 control', 'proportional-integral', 'TCDS']],
    tryit: 'In flight, move the lever to IDLE and watch N2 fall away.' }
];
let cur = 0;
function showPart(i) {
  cur = (i + PARTS.length) % PARTS.length;
  const p = PARTS[cur];
  $('#pKick').textContent = `Part ${cur + 1} of ${PARTS.length} · ${p.mod}`;
  $('#pTitle').textContent = p.name;
  const rows = p.facts.map(([k, v, s]) => `<tr><td>${k}<span class="src ${s === 'TCDS' ? 'tcds' : ''}">${s}</span></td><td>${v}</td></tr>`).join('');
  $('#pBody').innerHTML = `${p.body}<table class="facts">${rows}</table><p class="try"><b>Try it:</b> ${p.tryit}</p>`;
  document.querySelectorAll('#parts .chip').forEach((c, j) => c.setAttribute('aria-pressed', j === cur ? 'true' : 'false'));
}
$('#parts').innerHTML = PARTS.map((p, i) => `<button type="button" class="chip" data-i="${i}" aria-pressed="false"><b>${i + 1}</b>${p.name}</button>`).join('');
$('#parts').addEventListener('click', e => { const b = e.target.closest('.chip'); if (b) showPart(+b.dataset.i); });
$('#prev').onclick = () => showPart(cur - 1);
$('#next').onclick = () => showPart(cur + 1);

/* ---------- Theme tokens for canvas ---------- */
let K = {};
function readTokens() {
  const cs = getComputedStyle(document.documentElement);
  ['paper', 'panel', 'face', 'ink', 'muted', 'line', 'accent', 'metal', 'metal2', 'rotor', 'gas', 'ok', 'caution', 'warn', 'fuel']
    .forEach(k => K[k] = cs.getPropertyValue('--' + k).trim());
  K.fmono = cs.getPropertyValue('--font-mono').trim(); K.fdisp = cs.getPropertyValue('--font-display').trim(); K.fbody = cs.getPropertyValue('--font-body').trim();
}
readTokens();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readTokens);
new MutationObserver(readTokens).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class', 'style'] });

function fit(cv) {
  const r = cv.getBoundingClientRect();
  const d = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(r.width * d)), h = Math.max(1, Math.round(r.height * d));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(d, 0, 0, d, 0, 0);
  return { ctx, w: r.width, h: r.height };
}

/* ---------- Temperature colour ---------- */
const TSTOPS = [[-40, [64, 110, 214]], [100, [46, 160, 170]], [300, [214, 170, 52]], [600, [236, 118, 40]], [900, [218, 58, 40]], [1250, [176, 24, 96]]];
function tcol(c, a = 1) {
  c = clamp(c, -40, 1250);
  for (let i = 1; i < TSTOPS.length; i++) {
    if (c <= TSTOPS[i][0]) {
      const [c0, k0] = TSTOPS[i - 1], [c1, k1] = TSTOPS[i];
      const t = (c - c0) / (c1 - c0);
      return `rgba(${Math.round(lerp(k0[0], k1[0], t))},${Math.round(lerp(k0[1], k1[1], t))},${Math.round(lerp(k0[2], k1[2], t))},${a})`;
    }
  }
  return `rgba(176,24,96,${a})`;
}

/* ---------- Engine geometry (mm, x along axis, r radial; r<0 = lower half outside view) ---------- */
const WX0 = -70, WW = 1270, WR1 = 345, WH = 665;
let sc = 1;
const X = x => (x - WX0) * sc, Y = r => (WR1 - r) * sc;
const OUTER = [[-30, 195], [-10, 188], [60, 172], [160, 150], [262, 138], [338, 138], [380, 160], [425, 205], [445, 222], [640, 222], [660, 135], [720, 132], [728, 148], [792, 148]];
const G = {
  intake: [[-30, 62], [80, 68], [262, 70], [262, 138], [160, 150], [60, 172], [-10, 188], [-30, 195]],
  axial: [[262, 70], [338, 72], [338, 138], [262, 138]],
  imp: [[338, 72], [372, 90], [410, 140], [428, 190], [445, 192], [445, 222], [425, 205], [380, 160], [338, 138]],
  plenum: [[445, 100], [640, 100], [640, 222], [445, 222]],
  liner: [[478, 122], [602, 122], [612, 132], [612, 198], [602, 206], [478, 206], [468, 186], [468, 142]],
  noz: [[612, 132], [612, 198], [650, 130], [650, 62]],
  ggt: [[650, 62], [720, 60], [720, 132], [660, 135], [650, 130]],
  pt: [[720, 60], [728, 52], [792, 52], [792, 148], [728, 148], [720, 132]],
  exh: [[792, 52], [950, 58], [983, 110], [990, 345], [875, 345], [860, 200], [792, 148]]
};
const M = {
  nose: [[-30, 0], [-30, 62], [80, 68], [262, 70], [262, 0]],
  axialDisk: [[262, 0], [262, 70], [338, 72], [338, 0]],
  impHub: [[338, 0], [338, 72], [372, 90], [410, 140], [428, 190], [440, 190], [440, 0]],
  diff: [[428, 190], [445, 192], [445, 100], [440, 100], [440, 190]],
  inner: [[440, 0], [440, 100], [640, 100], [650, 62], [650, 0]],
  gg: [[652, 0], [652, 60], [718, 58], [718, 0]],
  ptd: [[730, 0], [730, 50], [790, 50], [790, 0]],
  rear: [[790, 0], [790, 52], [950, 58], [983, 110], [1000, 110], [1000, 0]]
};
const HIT = [
  { id: 'tot', x0: 714, x1: 734, r0: 80, r1: 165 },
  { id: 'fcu', x0: 135, x1: 265, r0: -232, r1: -155 },
  { id: 'intake', x0: -35, x1: 262, r0: 55, r1: 200 },
  { id: 'agb', x0: -35, x1: 262, r0: 0, r1: 55 },
  { id: 'axial', x0: 262, x1: 338, r0: 55, r1: 145 },
  { id: 'centrifugal', x0: 338, x1: 446, r0: 55, r1: 226 },
  { id: 'combustor', x0: 446, x1: 640, r0: 20, r1: 236 },
  { id: 'ggturbine', x0: 640, x1: 719, r0: 0, r1: 140 },
  { id: 'pt', x0: 719, x1: 794, r0: 0, r1: 152 },
  { id: 'exhaust', x0: 794, x1: 1000, r0: 52, r1: 345 },
  { id: 'rgb', x0: 1000, x1: 1190, r0: -300, r1: 60 },
  { id: 'agb', x0: -70, x1: 1000, r0: -275, r1: -225 },
  { id: 'agb', x0: -35, x1: 135, r0: -225, r1: -150 }
];
const BADGE = {
  intake: [40, 128], axial: [283, 162], centrifugal: [366, 188], combustor: [540, 252], ggturbine: [684, 162],
  tot: [724, 190], pt: [770, 176], exhaust: [930, 262], rgb: [1150, 72], agb: [420, -268], fcu: [242, -194]
};
const partIndex = id => PARTS.findIndex(p => p.id === id);

function poly(ctx, pts) { ctx.beginPath(); pts.forEach(([x, r], i) => i ? ctx.lineTo(X(x), Y(r)) : ctx.moveTo(X(x), Y(r))); }
function fillPoly(ctx, pts, fill, stroke, lw = 1) { poly(ctx, pts); ctx.closePath(); if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); } }
function line(ctx, pts, stroke, lw) { poly(ctx, pts); ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); }
function rectW(ctx, x0, x1, r0, r1, fill, stroke, lw = 1) { ctx.beginPath(); ctx.rect(X(x0), Y(r1), (x1 - x0) * sc, (r1 - r0) * sc); if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); } }

/* ---------- Particles ---------- */
function mkPath(entry) {
  const pts = [[-40, 140, 'T0'], [60, 120, 'T1'], [200, 104, 'T1'], [262, 104, 'T1'], [338, 105, 'T15'], [380, 125, 'Tm'], [415, 170, 'T2'], [436, 205, 'T2'],
    [460, 212, 'T2'], [entry, 212, 'T2'], [entry + 12, 165, 'TF'], [605, 165, 'TF'], [640, 100, 'T4'], [652, 95, 'T4'], [720, 95, 'T45'], [756, 100, 'T45'],
    [792, 100, 'T5'], [860, 120, 'T5'], [930, 190, 'T5'], [935, 350, 'T5']];
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  return { pts, cum, L: cum[cum.length - 1] };
}
const PATHS = [mkPath(495), mkPath(535), mkPath(578)];
const NP = reduceMotion ? 50 : 150;
const parts = Array.from({ length: NP }, () => ({ p: (Math.random() * 3) | 0, s: Math.random() * 1400, j: Math.random() * 16 - 8 }));
const spray = [];
function pathAt(P, s) {
  let i = 1; while (i < P.cum.length - 1 && P.cum[i] < s) i++;
  const t = clamp((s - P.cum[i - 1]) / (P.cum[i] - P.cum[i - 1]), 0, 1);
  const a = P.pts[i - 1], b = P.pts[i];
  return [lerp(a[0], b[0], t), lerp(a[1], b[1], t), a[2], b[2], t];
}
function tempKey(k, st, comb) {
  const v = { T0: st[0].T, T1: st[1].T, T15: st[2].T, Tm: (st[2].T + st[3].T) / 2, T2: st[3].T, TF: comb ? st[4].T + 220 : st[3].T, T4: st[4].T, T45: st[5].T, T5: st[6].T };
  return v[k] - 273.15;
}

/* ---------- Draw engine ---------- */
const cvE = $('#engine');
let hover = null, phGG = 0, phPT = 0, phOut = 0;
function bladeRow(ctx, x0, x1, r0, r1, phase, rotating) {
  const n = 7;
  ctx.lineWidth = Math.max(1, 1.6 * sc);
  if (rotating) {
    rectW(ctx, x0, x1, r0, r1, K.rotor + '55');
    ctx.strokeStyle = K.ink;
    for (let i = 0; i < n; i++) {
      const r = r0 + (((i / n) + phase) % 1) * (r1 - r0);
      ctx.beginPath(); ctx.moveTo(X(x0 + 1), Y(Math.min(r, r1))); ctx.lineTo(X(x1 - 1), Y(Math.min(r + 7, r1))); ctx.stroke();
    }
  } else {
    ctx.strokeStyle = K.muted;
    for (let i = 0; i < n; i++) {
      const r = r0 + (i + 0.5) / n * (r1 - r0);
      ctx.beginPath(); ctx.moveTo(X(x0 + 1), Y(r + 4)); ctx.lineTo(X(x1 - 1), Y(r - 4)); ctx.stroke();
    }
  }
}
function gear(ctx, cx, cy, R, teeth, ang, fill, stroke) {
  const th = Math.max(2.5, R * 0.12);
  ctx.beginPath();
  for (let i = 0; i < teeth * 2; i++) {
    const a0 = ang + i * Math.PI / teeth, a1 = a0 + Math.PI / teeth;
    const rr = i % 2 ? R - th / 2 : R + th / 2;
    ctx.lineTo(cx + Math.cos(a0) * rr, cy + Math.sin(a0) * rr);
    ctx.lineTo(cx + Math.cos(a1) * rr, cy + Math.sin(a1) * rr);
  }
  ctx.closePath(); ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke();
  ctx.beginPath(); ctx.arc(cx, cy, R * 0.28, 0, Math.PI * 2); ctx.fillStyle = stroke; ctx.globalAlpha = .25; ctx.fill(); ctx.globalAlpha = 1;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(ang) * R * 0.75, cy + Math.sin(ang) * R * 0.75); ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke();
}

function drawEngine(dtv) {
  const { ctx, w } = fit(cvE);
  sc = w / WW;
  ctx.clearRect(0, 0, w, WH * sc + 2);
  const st = TH.st, tint = $('#tTint').checked;
  const regionT = {
    intake: st[1].T, axial: (st[1].T + st[2].T) / 2, imp: (st[2].T + st[3].T) / 2, plenum: st[3].T,
    liner: S.comb ? st[4].T + 150 : st[3].T, noz: st[4].T, ggt: (st[4].T + st[5].T) / 2, pt: (st[5].T + st[6].T) / 2, exh: st[6].T
  };

  // centreline
  ctx.setLineDash([10 * sc, 6 * sc]); line(ctx, [[-60, 0], [1190, 0]], K.muted, 1); ctx.setLineDash([]);

  // lower half: outside view
  const low = [[-30, 0], ...OUTER.map(([x, r]) => [x, -r]), [860, -150], [1000, -150], [1000, 0]];
  const grad = ctx.createLinearGradient(0, Y(0), 0, Y(-222));
  grad.addColorStop(0, K.metal); grad.addColorStop(1, K.metal2);
  fillPoly(ctx, low, grad, K.ink, 1.2);
  ctx.save(); poly(ctx, low); ctx.closePath(); ctx.clip();
  ctx.strokeStyle = K.ink; ctx.globalAlpha = .35; ctx.lineWidth = 1;
  [262, 338, 720, 792].forEach(x => { ctx.beginPath(); ctx.moveTo(X(x), Y(0)); ctx.lineTo(X(x), Y(-240)); ctx.stroke(); });
  ctx.globalAlpha = 1; ctx.restore();
  ctx.fillStyle = K.ink; ctx.font = `600 ${Math.max(9, 12 * sc)}px ${K.fmono}`;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  [['M01', 115], ['M02', 300], ['M03', 530], ['M04', 870]].forEach(([t, x]) => ctx.fillText(t, X(x), Y(-60)));

  // gas path regions
  for (const k in G) fillPoly(ctx, G[k], tint ? tcol(regionT[k] - 273.15, 0.32) : K.gas);

  // metal in section
  fillPoly(ctx, M.nose, K.metal, K.ink, 1);
  fillPoly(ctx, M.inner, K.metal, K.ink, 1);
  fillPoly(ctx, M.rear, K.metal, K.ink, 1);
  fillPoly(ctx, M.diff, K.metal2, K.ink, 1);
  fillPoly(ctx, M.axialDisk, K.rotor, K.ink, 1);
  fillPoly(ctx, M.impHub, K.rotor, K.ink, 1);
  fillPoly(ctx, M.gg, K.rotor, K.ink, 1);
  fillPoly(ctx, M.ptd, K.rotor, K.ink, 1);
  rectW(ctx, 250, 720, 0, 20, K.rotor, K.ink, 1); // GG shaft
  rectW(ctx, 790, 1095, 0, 14, K.rotor, K.ink, 1); // PT shaft
  // shaft motion marks
  ctx.save(); ctx.strokeStyle = K.face; ctx.globalAlpha = .6; ctx.lineWidth = 1.5;
  ctx.setLineDash([6 * sc, 10 * sc]); ctx.lineDashOffset = -phGG * 16 * sc * 6;
  ctx.beginPath(); ctx.moveTo(X(250), Y(10)); ctx.lineTo(X(720), Y(10)); ctx.stroke();
  ctx.lineDashOffset = -phPT * 16 * sc * 6;
  ctx.beginPath(); ctx.moveTo(X(790), Y(7)); ctx.lineTo(X(1095), Y(7)); ctx.stroke();
  ctx.restore();
  // accessory gearbox gears (section hint)
  ctx.save(); ctx.translate(0, 0);
  gear(ctx, X(70), Y(30), 22 * sc, 12, phGG * 2 * Math.PI * 0.3, K.metal2, K.ink);
  gear(ctx, X(165), Y(26), 16 * sc, 9, -phGG * 2 * Math.PI * 0.4, K.metal2, K.ink);
  ctx.restore();
  // impeller vanes
  ctx.strokeStyle = K.ink; ctx.lineWidth = 1.2;
  [[0.35], [0.7]].forEach(([f]) => {
    ctx.beginPath(); ctx.moveTo(X(lerp(338, 345, f)), Y(lerp(72, 138, f)));
    ctx.quadraticCurveTo(X(lerp(395, 382, f)), Y(lerp(110, 150, f)), X(lerp(430, 428, f)), Y(lerp(192, 210, f))); ctx.stroke();
  });

  // blade rows
  bladeRow(ctx, 270, 296, 72, 136, phGG, true);
  bladeRow(ctx, 308, 330, 72, 137, 0, false);
  bladeRow(ctx, 655, 664, 64, 130, 0, false);
  bladeRow(ctx, 667, 681, 62, 131, phGG, true);
  bladeRow(ctx, 688, 696, 61, 131, 0, false);
  bladeRow(ctx, 700, 714, 60, 132, phGG + 0.33, true);
  bladeRow(ctx, 733, 743, 54, 146, 0, false);
  bladeRow(ctx, 752, 772, 53, 147, phPT, true);

  // combustor liner
  ctx.setLineDash([9 * sc, 4 * sc]); fillPoly(ctx, G.liner, null, K.ink, 1.8); ctx.setLineDash([]);
  // injection wheel and igniter
  rectW(ctx, 538, 552, 20, 46, K.rotor, K.ink, 1);
  rectW(ctx, 500, 508, 206, 236, K.ink);
  // casing
  line(ctx, OUTER, K.metal2, 9 * sc);
  line(ctx, OUTER, K.ink, 2.2);
  line(ctx, [[792, 148], [860, 200], [875, 345]], K.ink, 2.2);
  line(ctx, [[792, 52], [950, 58], [983, 110], [990, 345]], K.ink, 2.2);
  line(ctx, [[-30, 62], [80, 68], [262, 70]], K.ink, 1.5);
  // TOT probe
  rectW(ctx, 721, 727, 82, 160, K.ink);
  ctx.beginPath(); ctx.arc(X(724), Y(82), 3.5 * sc + 1, 0, Math.PI * 2); ctx.fillStyle = K.warn; ctx.fill();

  // flame glow
  if (S.comb) {
    const k = clamp(TH.Wf / 200, 0.15, 1.2);
    const g2 = ctx.createRadialGradient(X(560), Y(165), 2, X(560), Y(165), 75 * sc * (0.6 + k));
    g2.addColorStop(0, `rgba(255,170,60,${0.55 * Math.min(1, k + 0.3)})`); g2.addColorStop(1, 'rgba(255,120,30,0)');
    ctx.save(); poly(ctx, G.liner); ctx.closePath(); ctx.clip(); ctx.fillStyle = g2; ctx.fillRect(X(460), Y(215), 160 * sc, 100 * sc); ctx.restore();
  }

  // transmission tube, flange, AGB lower housing, RGB, FCU
  fillPoly(ctx, [[-30, -150], [135, -150], [135, -258], [-30, -258]], K.metal, K.ink, 1.2);
  rectW(ctx, -48, 1095, -249, -231, K.metal2, K.ink, 1.2);
  ctx.save(); ctx.strokeStyle = K.face; ctx.globalAlpha = .7; ctx.lineWidth = 1.5; ctx.setLineDash([5 * sc, 9 * sc]); ctx.lineDashOffset = -phOut * 14 * sc * 6;
  ctx.beginPath(); ctx.moveTo(X(-48), Y(-240)); ctx.lineTo(X(1050), Y(-240)); ctx.stroke(); ctx.restore();
  rectW(ctx, -64, -48, -268, -212, K.metal2, K.ink, 1.2);
  // RGB housing
  ctx.beginPath(); const rx = X(1003), ry = Y(42), rw = 184 * sc, rh = 342 * sc, rr = 14 * sc;
  ctx.moveTo(rx + rr, ry); ctx.arcTo(rx + rw, ry, rx + rw, ry + rh, rr); ctx.arcTo(rx + rw, ry + rh, rx, ry + rh, rr); ctx.arcTo(rx, ry + rh, rx, ry, rr); ctx.arcTo(rx, ry, rx + rw, ry, rr); ctx.closePath();
  ctx.fillStyle = K.metal; ctx.fill(); ctx.strokeStyle = K.ink; ctx.lineWidth = 1.2; ctx.stroke();
  const aP = phPT * 2 * Math.PI * 0.25;
  gear(ctx, X(1095), Y(0), 20 * sc, 12, aP, K.rotor, K.ink);
  gear(ctx, X(1095), Y(-104), 84 * sc, 50, -aP * 12 / 50 + Math.PI / 50, K.metal2, K.ink);
  gear(ctx, X(1095), Y(-240), 52 * sc, 31, aP * 12 / 31, K.rotor, K.ink);
  // FCU
  ctx.fillStyle = K.ink; ctx.textAlign = 'center'; ctx.font = `600 ${Math.max(9, 12 * sc)}px ${K.fmono}`;
  ctx.fillText('M05', X(1165), Y(-284));
  rectW(ctx, 140, 260, -226, -160, K.face, K.ink, 1.4);
  ctx.fillStyle = K.ink; ctx.font = `700 ${Math.max(10, 14 * sc)}px ${K.fdisp}`;
  ctx.textAlign = 'center'; ctx.fillText('FCU', X(190), Y(-193));
  // fuel line
  ctx.save(); ctx.setLineDash([6 * sc, 4 * sc]); ctx.lineDashOffset = S.comb ? -S.t * 30 : 0;
  line(ctx, [[260, -192], [500, -192], [545, -30], [545, 18]], S.lever === 'off' ? K.muted : K.fuel, 2);
  ctx.restore();
  // labels on outer view
  ctx.fillStyle = K.muted; ctx.font = `${Math.max(9, 11 * sc)}px ${K.fbody}`;
  ctx.textAlign = 'left'; ctx.fillText('Output to main gearbox ←', X(-60), Y(-292));
  ctx.textAlign = 'center'; ctx.fillText('Exhaust ↑', X(932), Y(330));

  // particles
  if ($('#tFlow').checked) {
    const v = 560 * TH.mdot / 2.5;
    const alpha = clamp(TH.mdot / 0.15, 0, 1);
    if (alpha > 0.01) {
      const rad = Math.max(1.5, 2.6 * sc);
      for (const q of parts) {
        const P = PATHS[q.p];
        q.s += v * dtv;
        if (q.s > P.L) { q.s -= P.L; q.p = (Math.random() * 3) | 0; q.j = Math.random() * 16 - 8; }
        const [x, r, ka, kb, t] = pathAt(P, q.s);
        const c = lerp(tempKey(ka, st, S.comb), tempKey(kb, st, S.comb), t);
        const jit = q.j * (x > 640 && x < 800 ? 0.5 : 1);
        ctx.beginPath(); ctx.arc(X(x), Y(r + jit), rad, 0, Math.PI * 2); ctx.fillStyle = tcol(c, alpha); ctx.fill();
      }
    }
    // fuel spray
    if (S.comb) {
      const rate = TH.Wf / 4 * dtv;
      for (let i = 0; i < Math.floor(rate) + (Math.random() < rate % 1 ? 1 : 0); i++)
        spray.push({ x: 545 + Math.random() * 6 - 3, r: 46, vr: 160 + Math.random() * 120, vx: Math.random() * 50 - 15 });
    }
    for (let i = spray.length - 1; i >= 0; i--) {
      const f = spray[i]; f.r += f.vr * dtv; f.x += f.vx * dtv;
      if (f.r > 175 || spray.length > 160) { spray.splice(i, 1); continue; }
      ctx.beginPath(); ctx.arc(X(f.x), Y(f.r), Math.max(1.2, 1.9 * sc), 0, Math.PI * 2); ctx.fillStyle = K.fuel; ctx.globalAlpha = 1 - (f.r - 46) / 140; ctx.fill(); ctx.globalAlpha = 1;
    }
  }

  // highlight
  const sel = PARTS[cur].id;
  [sel, hover].forEach((id, k) => {
    if (!id) return;
    const boxes = HIT.filter(h => h.id === id);
    ctx.save(); ctx.strokeStyle = K.accent; ctx.lineWidth = k ? 1.5 : 2.2; ctx.setLineDash(k ? [4, 4] : []);
    boxes.forEach(b => { ctx.strokeRect(X(b.x0), Y(b.r1), (b.x1 - b.x0) * sc, (b.r1 - b.r0) * sc); });
    ctx.restore();
  });

  // badges
  if ($('#tNums').checked) {
    const br = Math.max(8, 11 * sc);
    ctx.font = `600 ${Math.max(9, 11.5 * sc)}px ${K.fmono}`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    PARTS.forEach((p, i) => {
      const [x, r] = BADGE[p.id];
      ctx.beginPath(); ctx.arc(X(x), Y(r), br, 0, Math.PI * 2);
      ctx.fillStyle = i === cur ? K.accent : K.ink; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = K.face; ctx.stroke();
      ctx.fillStyle = K.face; ctx.fillText(String(i + 1), X(x), Y(r) + 0.5);
    });
  }
}
function hitTest(ev) {
  const r = cvE.getBoundingClientRect();
  const x = (ev.clientX - r.left) / sc + WX0, rr = WR1 - (ev.clientY - r.top) / sc;
  for (const p of PARTS) { const [bx, by] = BADGE[p.id]; if (Math.hypot(bx - x, by - rr) < 16) return p.id; }
  const h = HIT.find(b => x >= b.x0 && x <= b.x1 && rr >= b.r0 && rr <= b.r1);
  return h ? h.id : null;
}
cvE.addEventListener('mousemove', e => { hover = hitTest(e); cvE.style.cursor = hover ? 'pointer' : 'crosshair'; });
cvE.addEventListener('mouseleave', () => { hover = null; });
cvE.addEventListener('click', e => { const id = hitTest(e); if (id) showPart(partIndex(id)); });

/* ---------- Gauges ---------- */
const GAUGES = [
  { cv: $('#gN1'), label: 'N1', unit: '%', min: 0, max: 110, step: 10, f: 1,
    arcs: [[67, 98, 'ok'], [98, 101.9, 'caution'], [101.9, 110, 'warn']], red: [101.9], marks: [107.5], val: () => [S.N1] },
  { cv: $('#gN2'), label: 'N2 · NR', unit: '%', min: 0, max: 125, step: 25, f: 1,
    arcs: [[86, 96, 'caution'], [96, 104, 'ok'], [104, 108.5, 'caution'], [108.5, 125, 'warn']], red: [108.5], marks: [120.5], val: () => [S.N2, S.NR] },
  { cv: $('#gTOT'), label: 'TOT', unit: '°C', min: 0, max: 1000, step: 200, f: 0,
    arcs: [[400, 795, 'ok'], [795, 845, 'caution'], [845, 1000, 'warn']], red: [845], marks: [865], val: () => [S.TOT] },
  { cv: $('#gQ'), label: 'Torque', unit: 'm·daN', min: 0, max: 100, step: 20, f: 1,
    arcs: [[0, 83, 'ok'], [83, 91.5, 'caution'], [91.5, 100, 'warn']], red: [83], marks: [91.5], val: () => [S.q] },
  { cv: $('#gOP'), label: 'Oil press', unit: 'hPa', min: 0, max: 6000, step: 1000, f: 0, k: true,
    arcs: [[0, 1300, 'warn'], [1300, 1800, 'caution'], [1800, 5000, 'ok'], [5000, 6000, 'warn']], red: [5000], marks: [], val: () => [S.oilP] },
  { cv: $('#gOT'), label: 'Oil temp', unit: '°C', min: -40, max: 140, step: 40, f: 0,
    arcs: [[-40, 0, 'caution'], [0, 115, 'ok'], [115, 140, 'warn']], red: [115], marks: [], val: () => [S.oilT] }
];
function drawGauge(g) {
  const { ctx, w, h } = fit(g.cv);
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 3;
  const a0 = Math.PI * 0.75, sw = Math.PI * 1.5;
  const ang = v => a0 + clamp((v - g.min) / (g.max - g.min), 0, 1) * sw;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.fillStyle = K.face; ctx.fill(); ctx.strokeStyle = K.line; ctx.lineWidth = 1; ctx.stroke();
  const aw = Math.max(4, R * 0.07);
  g.arcs.forEach(([f, t, c]) => { ctx.beginPath(); ctx.arc(cx, cy, R - aw / 2 - 4, ang(f), ang(t)); ctx.lineWidth = aw; ctx.strokeStyle = K[c]; ctx.stroke(); });
  const fs = Math.max(8, R * 0.13);
  ctx.font = `${fs}px ${K.fmono}`;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillStyle = K.muted; ctx.strokeStyle = K.ink; ctx.lineWidth = 1;
  for (let v = g.min; v <= g.max + 1e-6; v += g.step) {
    const a = ang(v), c = Math.cos(a), s = Math.sin(a);
    ctx.beginPath(); ctx.moveTo(cx + c * (R - aw - 6), cy + s * (R - aw - 6)); ctx.lineTo(cx + c * (R - aw - 10), cy + s * (R - aw - 10)); ctx.stroke();
    const lbl = g.k ? String(v / 1000) : String(v);
    ctx.fillText(lbl, cx + c * (R - aw - 14 - fs * 1.15), cy + s * (R - aw - 14 - fs * 1.15));
  }
  g.red.forEach(v => { const a = ang(v); ctx.beginPath(); ctx.moveTo(cx + Math.cos(a) * (R - 1), cy + Math.sin(a) * (R - 1)); ctx.lineTo(cx + Math.cos(a) * (R - aw - 10), cy + Math.sin(a) * (R - aw - 10)); ctx.strokeStyle = K.warn; ctx.lineWidth = 3; ctx.stroke(); });
  g.marks.forEach(v => { const a = ang(v); ctx.beginPath(); ctx.arc(cx + Math.cos(a) * (R - aw / 2 - 4), cy + Math.sin(a) * (R - aw / 2 - 4), aw * 0.55, 0, Math.PI * 2); ctx.fillStyle = K.face; ctx.fill(); ctx.strokeStyle = K.warn; ctx.lineWidth = 1.5; ctx.stroke(); });
  const vals = g.val();
  vals.forEach((v, i) => {
    const a = ang(v);
    ctx.beginPath(); ctx.moveTo(cx - Math.cos(a) * R * 0.1, cy - Math.sin(a) * R * 0.1); ctx.lineTo(cx + Math.cos(a) * (R - aw - 8), cy + Math.sin(a) * (R - aw - 8));
    ctx.strokeStyle = i ? K.accent : K.ink; ctx.lineWidth = i ? 2 : 3; ctx.lineCap = 'round'; ctx.stroke(); ctx.lineCap = 'butt';
  });
  ctx.beginPath(); ctx.arc(cx, cy, R * 0.06, 0, Math.PI * 2); ctx.fillStyle = K.ink; ctx.fill();
  ctx.fillStyle = K.muted; ctx.font = `600 ${Math.max(10, R * 0.16)}px ${K.fdisp}`;
  ctx.fillText(g.label.toUpperCase(), cx, cy + R * 0.36);
  ctx.fillStyle = K.ink; ctx.font = `600 ${Math.max(10, R * 0.17)}px ${K.fmono}`;
  const txt = vals.length > 1 ? `${vals[0].toFixed(0)}/${vals[1].toFixed(0)}` : vals[0].toFixed(g.f);
  ctx.fillText(txt, cx, cy + R * 0.6);
  ctx.fillStyle = K.muted; ctx.font = `${Math.max(8, R * 0.11)}px ${K.fmono}`;
  ctx.fillText(g.unit, cx, cy + R * 0.8);
}

/* ---------- T–s chart ---------- */
const cvTs = $('#ts');
function sOf(T, P) { return 1.1 * Math.log(T / 288.15) - 0.287 * Math.log(P / 1.01325); }
function cyclePts(th, comb) {
  const s = th.st, out = [];
  const add = (st, lab) => out.push({ s: sOf(st.T, st.P), T: st.T - 273.15, lab });
  add(s[1], '1'); add(s[2], ''); add(s[3], '2');
  if (comb) for (let i = 1; i < 8; i++) { const t = i / 8; const T = lerp(s[3].T, s[4].T, t), P = lerp(s[3].P, s[4].P, t); out.push({ s: sOf(T, P), T: T - 273.15, lab: '' }); }
  add(s[4], comb ? '4' : ''); add(s[5], comb ? '4.5' : ''); add(s[6], comb ? '5' : '');
  return out;
}
function drawTs() {
  const { ctx, w, h } = fit(cvTs);
  ctx.clearRect(0, 0, w, h);
  const pl = 44, pr = 12, pt = 10, pb = 30;
  const ref = cyclePts(REF, true), now = cyclePts(TH, S.comb);
  const smin = -0.45, smax = 1.5, Tmin = -50, Tmax = 1250;
  const sx = s => pl + (s - smin) / (smax - smin) * (w - pl - pr);
  const ty = T => h - pb - (T - Tmin) / (Tmax - Tmin) * (h - pt - pb);
  const mono = K.fmono;
  ctx.font = `10px ${mono}`; ctx.fillStyle = K.muted; ctx.strokeStyle = K.line; ctx.lineWidth = 1;
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let T = 0; T <= 1200; T += 300) { ctx.beginPath(); ctx.moveTo(pl, ty(T)); ctx.lineTo(w - pr, ty(T)); ctx.stroke(); ctx.fillText(T + '°', pl - 6, ty(T)); }
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (let s = -0.4; s <= 1.21; s += 0.4) { ctx.beginPath(); ctx.moveTo(sx(s), pt); ctx.lineTo(sx(s), h - pb); ctx.stroke(); ctx.fillText(s.toFixed(1), sx(s), h - pb + 5); }
  ctx.fillText('entropy, kJ/kg·K (relative)', (pl + w - pr) / 2, h - 13);
  // TOT limit
  ctx.setLineDash([3, 3]); ctx.strokeStyle = K.warn; ctx.beginPath(); ctx.moveTo(pl, ty(845)); ctx.lineTo(w - pr, ty(845)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = K.warn; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillText('TOT limit 845 °C', pl + 4, ty(845) - 2);
  const path = (pts, col, lw, dash) => {
    ctx.beginPath(); pts.forEach((p, i) => i ? ctx.lineTo(sx(p.s), ty(p.T)) : ctx.moveTo(sx(p.s), ty(p.T)));
    ctx.setLineDash(dash || []); ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.stroke(); ctx.setLineDash([]);
  };
  path(ref, K.muted, 1.2, [4, 3]);
  path([ref[ref.length - 1], ref[0]], K.muted, 1, [1, 4]);
  path(now, K.accent, 2.4);
  if (S.comb) path([now[now.length - 1], now[0]], K.accent, 1, [2, 4]);
  ctx.font = `600 11px ${mono}`; ctx.textBaseline = 'middle';
  now.forEach(p => {
    if (!p.lab) return;
    ctx.beginPath(); ctx.arc(sx(p.s), ty(p.T), 3.5, 0, Math.PI * 2); ctx.fillStyle = tcol(p.T); ctx.fill(); ctx.strokeStyle = K.ink; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = K.ink; ctx.textAlign = p.lab === '5' ? 'right' : 'left';
    ctx.fillText(p.lab, sx(p.s) + (p.lab === '5' ? -7 : 6), ty(p.T) + (p.lab === '5' ? 8 : -7));
  });
}

/* ---------- Gear inset ---------- */
const cvG = $('#gears');
function drawGears() {
  const { ctx, w, h } = fit(cvG);
  ctx.clearRect(0, 0, w, h);
  const u = w / 300;
  const cy = h * 0.5, rp = 12 * u, ri = 30 * u, ro = 83 * u;
  const xp = 16 * u + rp, xi = xp + rp + ri, xo = xi + ri + ro;
  const aP = phPT * 2 * Math.PI * 0.25;
  gear(ctx, xo, cy, ro, 83, aP * 12 / 83, K.metal2, K.ink);
  gear(ctx, xi, cy, ri, 30, -aP * 12 / 30 + Math.PI / 30, K.metal, K.ink);
  gear(ctx, xp, cy, rp, 12, aP, K.rotor, K.ink);
  const mono = K.fmono;
  ctx.fillStyle = K.muted; ctx.font = `${Math.max(9, 10 * u)}px ${mono}`; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  ctx.fillText('pinion', xp, cy + ri + 6 * u); ctx.fillText('idler', xi, cy + ri + 6 * u);
  ctx.textBaseline = 'bottom'; ctx.fillText('output', xo, h - 2);
}

/* ---------- DOM readouts ---------- */
const annunEl = $('#annun');
annunEl.innerHTML = LAMPS.map(([n]) => `<div class="lamp" data-n="${n}">${n}</div>`).join('');
const lampEls = [...annunEl.children];
const fmt = (v, d = 0) => v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d });
function phaseText() {
  if (S.speed === 0) return ['Paused', 'Choose 1× or 3× to continue.'];
  if (!S.comb && S.starter) {
    if (S.lever === 'off') return ['Dry crank', `No fuel. The starter turns only the gas generator (N1 ${fmt(S.N1)}%) and the airflow cools the turbine.`];
    return ['Cranking', `The starter turns only the gas generator. Ignition and fuel come on at 10% N1. N1 is ${fmt(S.N1)}%.`];
  }
  if (S.comb && S.starting) {
    if (S.N1 < CUTOUT && !S.starter && S.N1 >= 25) return ['Slow start', `Starter released before cut-out. The turbine is accelerating the gas generator on its own, but slowly (N1 ${fmt(S.N1)}%). Watch TOT.`];
    if (S.N1 < CUTOUT && !S.starter) return ['Hung start', `Starter released before cut-out. Below about 25% N1 the turbine cannot accelerate the gas generator on its own (N1 ${fmt(S.N1)}%). Hold the starter, or cut the fuel.`];
    if (S.N1 < CUTOUT) return ['Light-off', `Combustion and starter accelerate the gas generator together. Keep holding the starter. Watch the TOT peak: start limit 865 °C. Peak so far ${fmt(S.startPeak)} °C.`];
    return ['Self-sustaining', `Starter cut out at 45% N1. The turbine now drives the compressor on its own and accelerates to idle.${S.startHeld ? ' Release the starter.' : ''}`];
  }
  if (S.comb) {
    if (S.startHeld) return ['Release starter', `The engine is running and the starter has cut out. Release the starter switch. N1 ${fmt(S.N1, 1)}%.`];
    if (S.lever === 'idle') return ['Ground idle', `The fuel control holds N1 near ${N1_IDLE}%. The free turbine and rotor turn slower than in flight (N2 ${fmt(S.N2)}%).`];
    if (S.N1 > N1_STOP - 0.3 && S.NR < 98) return ['Power-limited', 'N1 is at the fuel stop and cannot give more. The rotor is drooping. Lower the collective.'];
    if (S.TOT > C.TOT_TO) return ['Over-temp', `TOT ${fmt(S.TOT)} °C is above the 845 °C take-off limit. The fuel control does not protect TOT. Lower the collective.`];
    if (!S.coupled) return ['Re-engaging', 'The free turbine is spinning up to meet the rotor. The free wheel engages when the speeds match.'];
    return ['Governing', `The fuel control holds N2 at ${fmt(S.N2, 1)}% and sets N1 to ${fmt(S.N1, 1)}% to supply ${fmt(S.P)} kW.`];
  }
  if (S.airborne) return ['Flameout', `The free wheel lets the rotor (NR ${fmt(S.NR)}%) overrun the dead engine (N2 ${fmt(S.N2)}%). Lower the collective to hold rotor speed. Hold the starter to relight.`];
  if (S.N1 > 3) return ['Spooling down', `No fuel. N1 ${fmt(S.N1)}% and TOT ${fmt(S.TOT)} °C are falling.`];
  return ['Shut down', S.lever === 'off' ? 'Set the lever to IDLE, then press and hold the starter.' : 'Press and hold the starter to begin a start.'];
}
function updateDOM() {
  const [ph, msg] = phaseText();
  $('#phase').textContent = ph;
  $('#msg').textContent = (S.t < noteUntil && lastNote ? lastNote + '. ' : '') + msg;
  $('#simclock').textContent = 't ' + fmtT(S.t);
  const L = lampLevels();
  lampEls.forEach(el => { el.className = 'lamp ' + (L[el.dataset.n] || ''); });
  $('#rP').innerHTML = `${fmt(S.P)} <small>kW · ${fmt(S.P / C.P_TO * 100)}% of TO</small>`;
  $('#rOut').innerHTML = `${fmt(C.OUT_RPM * S.N2 / 100)} <small>rpm</small>`;
  $('#rN1').innerHTML = `${fmt(C.N1_RPM * S.N1 / 100)} <small>rpm</small>`;
  $('#rN2').innerHTML = `${fmt(C.N2_RPM * S.N2 / 100)} <small>rpm</small>`;
  $('#rWf').innerHTML = `${fmt(TH.Wf)} <small>kg/h · ${fmt(TH.Wf / 0.8)} L/h</small>`;
  $('#rM').innerHTML = `${fmt(TH.mdot, 2)} <small>kg/s</small>`;
  $('#rPR').innerHTML = `${fmt(TH.PR, 1)} <small>: 1</small>`;
  const eta = TH.Q > 1 ? S.P / TH.Q * 100 : 0;
  $('#rEta').innerHTML = `${fmt(eta)} <small>%</small>`;
  // energy
  const Q = Math.max(TH.Q, 1e-6), shaft = Math.min(S.P, Q);
  $('#eShaft').style.width = (TH.Q > 1 ? shaft / Q * 100 : 0) + '%';
  $('#eLoss').style.width = (TH.Q > 1 ? (Q - shaft) / Q * 100 : 0) + '%';
  $('#eShaftV').textContent = TH.Q > 1 ? `${fmt(shaft)} kW · ${fmt(shaft / Q * 100)}%` : '–';
  $('#eLossV').textContent = TH.Q > 1 ? `${fmt(Q - shaft)} kW · ${fmt((Q - shaft) / Q * 100)}%` : '–';
  const tw = TH.Wc + S.P;
  $('#tComp').style.width = (tw > 1 ? TH.Wc / tw * 100 : 0) + '%';
  $('#tShaft').style.width = (tw > 1 ? S.P / tw * 100 : 0) + '%';
  $('#tCompV').textContent = tw > 1 ? `${fmt(TH.Wc)} kW · ${fmt(TH.Wc / tw * 100)}%` : '–';
  $('#tShaftV').textContent = tw > 1 ? `${fmt(S.P)} kW · ${fmt(S.P / tw * 100)}%` : '–';
  const wPT = C.N2_RPM * S.N2 / 100 * 2 * Math.PI / 60;
  $('#qIn').textContent = S.P > 0 && S.N2 > 3 ? `${fmt(S.P * 1000 / wPT, 0)} N·m` : '–';
  $('#qOut').textContent = S.q > 0 ? `${fmt(S.q * 10, 0)} N·m (${fmt(S.q, 1)} m·daN)` : '–';
  // stations
  $('#stations tbody').innerHTML = TH.st.map(s => `<tr><td class="mono">${s.k}</td><td>${s.name}</td><td class="n">${s.P.toFixed(2)}</td><td class="n">${fmt(s.T - 273.15)}</td></tr>`).join('');
  // controls
  const btn = $('#btnStart');
  btn.textContent = !S.startHeld ? 'Hold to start' : S.starter ? 'Starter engaged' : 'Starter cut out · release';
  btn.classList.toggle('on', S.startHeld);
  btn.setAttribute('aria-pressed', String(S.startHeld));
  btn.disabled = !S.startHeld && !canStart();
  $('#startHint').textContent = S.startHeld ? (S.starter ? 'Hold until the starter cuts out at 45% N1.' : 'Cut out at 45% N1. Release the switch.')
    : S.comb && S.starting && S.N1 < CUTOUT ? 'Released before cut-out. Hold it again to finish the start.'
    : S.comb ? 'Engine running. Shut down (lever OFF) before starting again.'
    : S.N1 >= CUTOUT ? 'Wait for N1 to fall below 45%.'
    : S.lever === 'off' ? 'Lever is OFF: the starter will dry-crank with no fuel.'
    : S.hot ? 'Hot start armed. Be ready to pull the lever to OFF.' : 'Ignition and fuel come on automatically at 10% N1.';
  $('#btnFlameout').disabled = !S.comb;
  $('#preqOut').textContent = `${fmt(preq())} kW`;
  const a = amb();
  $('#airOut').textContent = `Air density ${fmt(a.de / a.th * 100)}% of sea-level standard`;
  const sig = E.join();
  if (sig !== expSig) { expSig = sig; renderExp(); }
  if (logDirty) { logDirty = false; renderLog(); }
}

/* ---------- Inputs ---------- */
function setSeg(id, v) { document.querySelectorAll(`#${id} button`).forEach(b => b.setAttribute('aria-checked', b.dataset.v === String(v) ? 'true' : 'false')); }
function syncInputs() {
  setSeg('lever', S.lever); setSeg('air', S.airborne ? 'air' : 'ground'); setSeg('speed', S.speed);
  $('#coll').value = S.coll; $('#collOut').textContent = S.coll + '%';
  $('#oat').value = S.oat; $('#oatOut').textContent = S.oat + ' °C';
  $('#alt').value = S.alt; $('#altOut').textContent = fmt(S.alt) + ' ft';
  $('#fHot').checked = S.hot; $('#fFoul').checked = S.foul; $('#fBleed').checked = S.bleed;
}
$('#lever').addEventListener('click', e => { const b = e.target.closest('button'); if (!b) return; S.lever = b.dataset.v; setSeg('lever', S.lever); });
$('#air').addEventListener('click', e => { const b = e.target.closest('button'); if (!b) return; S.airborne = b.dataset.v === 'air'; setSeg('air', b.dataset.v); });
$('#speed').addEventListener('click', e => { const b = e.target.closest('button'); if (!b) return; S.speed = +b.dataset.v; setSeg('speed', S.speed); });
$('#coll').addEventListener('input', e => { S.coll = +e.target.value; $('#collOut').textContent = S.coll + '%'; });
$('#oat').addEventListener('input', e => { S.oat = +e.target.value; $('#oatOut').textContent = S.oat + ' °C'; });
$('#alt').addEventListener('input', e => { S.alt = +e.target.value; $('#altOut').textContent = fmt(S.alt) + ' ft'; });
$('#fHot').addEventListener('change', e => { S.hot = e.target.checked; });
$('#fFoul').addEventListener('change', e => { S.foul = e.target.checked; });
$('#fBleed').addEventListener('change', e => { S.bleed = e.target.checked; });
// Momentary starter switch: pointer or Space/Enter held down engages it; releasing lets it go.
const btnStart = $('#btnStart');
// Available when shut down below cut-out speed, or to re-engage during a start that has not reached cut-out.
const canStart = () => S.N1 < CUTOUT && (!S.comb || S.starting);
function pressStarter() {
  if (S.startHeld || !canStart()) return;
  S.startHeld = true; S.cutout = false; updateDOM();
}
function releaseStarter() {
  if (!S.startHeld) return;
  S.startHeld = false; S.starter = false; note('Starter released'); updateDOM();
}
btnStart.addEventListener('pointerdown', e => {
  if (e.button !== 0) return;
  e.preventDefault(); btnStart.focus();
  try { btnStart.setPointerCapture(e.pointerId); } catch (_) {}
  pressStarter();
});
['pointerup', 'pointercancel', 'lostpointercapture'].forEach(t => btnStart.addEventListener(t, releaseStarter));
btnStart.addEventListener('keydown', e => { if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) { e.preventDefault(); pressStarter(); } });
btnStart.addEventListener('keyup', e => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); releaseStarter(); } });
btnStart.addEventListener('blur', releaseStarter);
btnStart.addEventListener('contextmenu', e => e.preventDefault());
window.addEventListener('blur', releaseStarter);
$('#btnFlameout').addEventListener('click', () => {
  if (!S.comb) return;
  S.comb = false; S.starting = false; S.flameoutAt = S.t; note('Flameout');
});
$('#btnReset').addEventListener('click', () => { resetState(); logItems.length = 0; logDirty = true; });
$('#clearLog').addEventListener('click', () => { logItems.length = 0; logDirty = true; });

/* ---------- Main loop ---------- */
resetState();
TH = thermoCalc(S.N1, S.TOT, amb().TK, amb().de, S.comb, S.bleed);
showPart(0);
renderExp();
let last = performance.now(), domAcc = 1, tsAcc = 1;
function frame(now) {
  const real = Math.min((now - last) / 1000, 0.1); last = now;
  const simDt = real * S.speed;
  let rem = simDt;
  while (rem > 1e-6) { const d = Math.min(rem, 0.02); step(d); rem -= d; }
  const a = amb();
  TH = thermoCalc(S.N1, S.TOT, a.TK, a.de, S.comb, S.bleed);
  // visual rotation: cycles per second, scaled down so motion stays readable
  if (!reduceMotion) {
    phGG = (phGG + simDt * 1.8 * S.N1 / 100) % 1000;
    phPT = (phPT + simDt * 1.8 * S.N2 / 100) % 1000;
    phOut = (phOut + simDt * 1.2 * S.N2 / 100) % 1000;
  }
  drawEngine(reduceMotion ? simDt * 0.4 : simDt);
  GAUGES.forEach(drawGauge);
  drawGears();
  domAcc += real; tsAcc += real;
  if (domAcc > 0.1) { domAcc = 0; updateDOM(); }
  if (tsAcc > 0.12) { tsAcc = 0; drawTs(); }
  requestAnimationFrame(frame);
}
updateDOM(); drawTs();
requestAnimationFrame(frame);
})();
