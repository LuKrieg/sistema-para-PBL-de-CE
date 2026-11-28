// POWERGuard - Lógica del Dashboard en Vivo (Web Serial API + Simulador)

let stepCapacitances = [60, 60, 60]; // uF por paso (D12, D11, D10)
const STEP_PINS_LABEL = ["D12", "D11", "D10"];
const MAX_HISTORY = 80;
const FP_TARGET = 0.95;
// P de entrada y FP de los escenarios de la propuesta; 0 % es motor energizado en vacío.
const SIM_MOTOR_POINTS = [
  { load: 0, p: 118 * 8.10 * 0.20, fp: 0.20 },
  { load: 25, p: 245.4, fp: 0.35 },
  { load: 50, p: 395.1, fp: 0.43 },
  { load: 75, p: 505.9, fp: 0.51 },
  { load: 100, p: 626.9, fp: 0.59 },
];
let lastSource = null;

let serialPort = null;
let serialWriter = null;
let simTimer = null;
let currentMode = "MANUAL"; // el firmware arranca en MANUAL con todos los pasos apagados

// Bloqueo de reconexión igual al firmware: t = R*C*ln(169.7 V / 50 V), mínimo 2 s
const DISCHARGE_R_OHMS = 100000;
const lockoutSeconds = (cUf) => Math.max(2.0, DISCHARGE_R_OHMS * cUf * 1e-6 * Math.log(169.7 / 50.0));

const CAL_MESSAGES = {
  CAL_OK: "Calibración de fase guardada en el Arduino.",
  CAL_SIN_CARGA: "No se pudo calibrar: no hay voltaje y corriente. Enciende la carga primero.",
  CAL_VALOR_INVALIDO: "Usa CAL:1.00 con una carga resistiva.",
  CAL_APAGAR_BANCO: "Selecciona MANUAL y apaga los tres pasos antes de calibrar.",
  CAL_FUERA_RANGO: "El ajuste de fase quedó fuera del rango admitido.",
};
let history = [];
let recordedRows = [];

// Estado de simulación local
const simState = {
  t: 0,
  loadPct: 0,
  voltage: 118,
  steps: [0, 0, 0],
  lockedUntil: [0, 0, 0],
  lastSwitchT: -10,
};

// Elementos del DOM
const elConnBadge = document.getElementById("conn-badge");
const elBtnConnect = document.getElementById("btn-connect");
const elBtnSim = document.getElementById("btn-sim");
const elBtnExport = document.getElementById("btn-export");
const elBtnAuto = document.getElementById("btn-auto");
const elBtnManual = document.getElementById("btn-manual");
const elStatusPill = document.getElementById("status-pill");
const elGaugeArc = document.getElementById("gauge-arc");
const elValFp = document.getElementById("val-fp");
const elValV = document.getElementById("val-vrms");
const elValI = document.getElementById("val-irms");
const elValP = document.getElementById("val-p");
const elValQ = document.getElementById("val-q");
const elValS = document.getElementById("val-s");
const elValCap = document.getElementById("val-cap");
const elSampleCount = document.getElementById("sample-count");
const elStepsContainer = document.getElementById("steps-container");
const elTriangleAngle = document.getElementById("triangle-angle");
const canvas = document.getElementById("live-chart");
const ctx = canvas.getContext("2d");
const triCanvas = document.getElementById("triangle-chart");
const triCtx = triCanvas.getContext("2d");
const elValPhase = document.getElementById("val-phase");
const elValCarga = document.getElementById("val-carga");
const elFpSource = document.getElementById("fp-source");
const elFpMeasured = document.getElementById("fp-measured");
const elIMeasured = document.getElementById("irms-measured");
const elBankDischarge = document.getElementById("bank-discharge");
const elFpEquation = document.getElementById("fp-equation");
const elMotorAngle = document.getElementById("motor-angle");
const elPhaseExplanation = document.getElementById("phase-explanation");
const elSimLoad = document.getElementById("sim-load");
const elSimLoadValue = document.getElementById("sim-load-value");
const elSimVoltage = document.getElementById("sim-voltage");
const elSimOperatingPoint = document.getElementById("sim-operating-point");
const elSimRecommendation = document.getElementById("sim-recommendation");
const elBtnSimRun = document.getElementById("btn-sim-run");

let testCounter = parseInt(localStorage.getItem("powerguard_prueba_num") || "1", 10);
function updateExportButtonLabel() {
  elBtnExport.textContent = `Exportar Prueba_${testCounter}.csv`;
}
updateExportButtonLabel();

// Tarjetas de los 3 pasos: se crean UNA vez y luego solo se actualizan.
// (Reconstruirlas en cada lectura hacía que el clic se perdiera: el botón desaparecía
// entre que se presionaba y se soltaba.)
let stepCardEls = null;
let lastSteps = [0, 0, 0];

function buildStepCards() {
  elStepsContainer.innerHTML = "";
  stepCardEls = [];
  for (let i = 0; i < stepCapacitances.length; i++) {
    const card = document.createElement("div");
    card.className = "step-card";
    card.innerHTML = `
      <div class="step-top">
        <span class="step-title">Paso ${i + 1} (${STEP_PINS_LABEL[i]})</span>
        <span class="step-state off">APAGADO</span>
      </div>
      <div class="step-cap">Capacitancia: <strong>${stepCapacitances[i].toFixed(1)} µF</strong></div>
      <div class="step-lock"></div>
      <button type="button" class="step-btn">Conectar</button>
    `;
    const btn = card.querySelector(".step-btn");
    btn.addEventListener("click", () => setStep(i, lastSteps[i] === 1 ? 0 : 1));
    elStepsContainer.appendChild(card);
    stepCardEls.push({
      card,
      state: card.querySelector(".step-state"),
      lock: card.querySelector(".step-lock"),
      btn,
    });
  }
}

function renderStepCards(steps = [0, 0, 0], lockedMs = [0, 0, 0]) {
  if (!stepCardEls) buildStepCards();
  lastSteps = [...steps];
  for (let i = 0; i < stepCardEls.length; i++) {
    const el = stepCardEls[i];
    const isOn = steps[i] === 1;
    const lockSec = (lockedMs[i] || 0) / 1000;
    el.card.classList.toggle("on", isOn);
    el.state.className = `step-state ${isOn ? "on" : "off"}`;
    el.state.textContent = isOn ? "CONECTADO" : "APAGADO";
    el.lock.textContent = lockSec > 0 && !isOn ? `Reconexión en ${lockSec.toFixed(1)} s` : "";
    const wantDisabled = lockSec > 0 && !isOn;
    if (el.btn.disabled !== wantDisabled) el.btn.disabled = wantDisabled;
    const label = isOn ? "Desconectar" : "Conectar";
    if (el.btn.textContent !== label) el.btn.textContent = label;
  }
}

function updateGauge(fp) {
  const clamped = Math.max(0, Math.min(1, fp));
  const angle = Math.PI * (1 - clamped); // pi -> 0
  const r = 100;
  const cx = 120;
  const cy = 120;
  const x = cx + r * Math.cos(angle);
  const y = cy - r * Math.sin(angle);
  const largeArc = 0;
  elGaugeArc.setAttribute("d", `M 20 120 A 100 100 0 ${largeArc} 1 ${x.toFixed(1)} ${y.toFixed(1)}`);

  if (clamped >= FP_TARGET) {
    elGaugeArc.setAttribute("stroke", "#10b981");
  } else if (clamped >= 0.85) {
    elGaugeArc.setAttribute("stroke", "#38bdf8");
  } else {
    elGaugeArc.setAttribute("stroke", "#f59e0b");
  }
  elValFp.textContent = fp == null ? "—" : fp.toFixed(3);
}

let lastTriangleData = { p: 0, q: 0, s: 0, fp: 1 };

function syncCanvasSize(c, context) {
  const rect = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const targetW = Math.max(100, Math.round(rect.width * dpr));
  const targetH = Math.max(100, Math.round(rect.height * dpr));
  if (c.width !== targetW || c.height !== targetH) {
    c.width = targetW;
    c.height = targetH;
  }
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { w: rect.width, h: rect.height };
}

function drawChart() {
  const { w, h } = syncCanvasSize(canvas, ctx);
  ctx.clearRect(0, 0, w, h);

  const padL = 42, padR = 42, padT = 18, padB = 24;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  // Línea de meta
  const y095 = padT + plotH * (1 - FP_TARGET);
  ctx.save();
  ctx.setLineDash([5, 5]);
  ctx.strokeStyle = "#10b981";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, y095); ctx.lineTo(w - padR, y095);
  ctx.stroke();
  ctx.restore();

  if (history.length < 2) return;

  // Curva de FP (0..1)
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  history.forEach((pt, idx) => {
    const x = padL + (idx / (MAX_HISTORY - 1)) * plotW;
    const y = padT + plotH * (1 - Math.max(0, Math.min(1, pt.fp)));
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Curva de Corriente (escala 0..12 A)
  ctx.strokeStyle = "#f59e0b";
  ctx.lineWidth = 1.8;
  ctx.beginPath();
  history.forEach((pt, idx) => {
    const x = padL + (idx / (MAX_HISTORY - 1)) * plotW;
    const y = padT + plotH * (1 - Math.max(0, Math.min(1, pt.irms / 12.0)));
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Etiquetas de ejes
  ctx.fillStyle = "#9ca3af";
  ctx.font = "11px Segoe UI, sans-serif";
  ctx.fillText("1.00", 8, padT + 4);
  ctx.fillText("0.50", 8, padT + plotH * 0.5);
  ctx.fillText("0.00", 8, padT + plotH);
  ctx.fillText("12A", w - 34, padT + 4);
  ctx.fillText("0A", w - 30, padT + plotH);
}

function drawPowerTriangle(p, q, s, fp) {
  lastTriangleData = { p, q, s, fp };
  const { w, h } = syncCanvasSize(triCanvas, triCtx);
  triCtx.clearRect(0, 0, w, h);

  const phiDeg = rad2deg(Math.atan2(q, p));
  elTriangleAngle.textContent = `φ = ${phiDeg.toFixed(1)}°`;

  const padL = 28, padR = 84, padT = 26, padB = 34;
  const maxW = Math.max(40, w - padL - padR);
  const maxH = Math.max(20, (h - padT - padB) / 2);

  if (s < 1.0) {
    triCtx.fillStyle = "#9ca3af";
    triCtx.font = "13px Segoe UI, sans-serif";
    triCtx.fillText("Esperando carga...", w / 2 - 55, h / 2);
    return;
  }

  const maxDim = Math.max(p, Math.abs(q), 1);
  const scale = Math.min(maxW / maxDim, maxH / maxDim);
  const px = Math.max(20, p * scale);
  const qy = q * scale;

  const x0 = padL;
  const y0 = (h + padT - padB) / 2;
  const x1 = x0 + px;
  const y1 = y0 - qy;

  // Relleno sutil del triángulo
  triCtx.fillStyle = "rgba(56, 189, 248, 0.08)";
  triCtx.beginPath();
  triCtx.moveTo(x0, y0);
  triCtx.lineTo(x1, y0);
  triCtx.lineTo(x1, y1);
  triCtx.closePath();
  triCtx.fill();

  // Cateto P (Activa)
  triCtx.strokeStyle = "#38bdf8";
  triCtx.lineWidth = 3;
  triCtx.beginPath();
  triCtx.moveTo(x0, y0);
  triCtx.lineTo(x1, y0);
  triCtx.stroke();

  // Cateto Q (Reactiva)
  triCtx.strokeStyle = "#f59e0b";
  triCtx.lineWidth = 3;
  triCtx.beginPath();
  triCtx.moveTo(x1, y0);
  triCtx.lineTo(x1, y1);
  triCtx.stroke();

  // Hipotenusa de la componente P + jQ
  triCtx.strokeStyle = fp >= FP_TARGET ? "#10b981" : "#e5e7eb";
  triCtx.lineWidth = 3;
  triCtx.beginPath();
  triCtx.moveTo(x0, y0);
  triCtx.lineTo(x1, y1);
  triCtx.stroke();

  // Etiquetas
  triCtx.font = "12px Segoe UI, sans-serif";
  triCtx.fillStyle = "#38bdf8";
  triCtx.fillText(`P = ${p.toFixed(0)} W`, x0 + px * 0.3, y0 + 18);

  triCtx.fillStyle = "#f59e0b";
  triCtx.fillText(`Q = ${q.toFixed(0)} var ${q < 0 ? "(cap.)" : "(ind.)"}`, x1 + 6, y0 - qy * 0.5);

  triCtx.fillStyle = fp >= FP_TARGET ? "#10b981" : "#e5e7eb";
  triCtx.fillText(`S = ${s.toFixed(0)} VA · |P+jQ| = ${Math.hypot(p, q).toFixed(0)} VA`, x0, 15);
}

// ---------- Asistente de compensación ----------
// Predice las combinaciones a partir de la carga sola y del residuo no activo.
const OMEGA = 2 * Math.PI * 60;
const ASSIST_TARGETS = [0.95, 0.99];
const elAssistLoad = document.getElementById("assist-load");
const elAssistRows = document.getElementById("assist-rows");
const elAssistNote = document.getElementById("assist-note");
let assistHist = [];
let assistLastCap = -1;
let assistCombos = {};

function capQvar(cUf, v) {
  return OMEGA * cUf * 1e-6 * v * v;
}

function predictCombos(p, qLoad, v, residual) {
  const out = [];
  for (let m = 0; m < 8; m++) {
    const bits = [m & 1, (m >> 1) & 1, (m >> 2) & 1];
    const c = bits.reduce((a, b, i) => a + b * stepCapacitances[i], 0);
    const qNet = qLoad - capQvar(c, v);
    const fp = p / Math.hypot(p, qNet, residual);
    out.push({ bits, c, qNet, fp });
  }
  return out;
}

function pickCombo(combos, target) {
  const sorted = [...combos].sort((a, b) => a.c - b.c || b.fp - a.fp);
  const ok = sorted.find((combo) => combo.fp >= target);
  if (ok) return { combo: ok, reaches: true };
  return { combo: sorted.reduce((best, combo) => combo.fp > best.fp ? combo : best), reaches: false };
}

function comboLabel(bits) {
  const names = bits.map((b, i) => (b ? `Paso ${i + 1}` : null)).filter(Boolean);
  return names.length ? names.join(" + ") : "ningún capacitor";
}

function updateAssistant(data) {
  const v = data.vrms || 0;
  if (!data.mc || !(data.p > 0) || v <= 0) {
    elAssistLoad.textContent = "Esperando carga…";
    elAssistRows.innerHTML = "";
    elAssistNote.textContent = "Enciende el motor para calcular qué capacitores conectar.";
    assistHist = [];
    assistCombos = {};
    return;
  }
  const capNow = data.cap_uf || 0;
  if (Math.abs(capNow - assistLastCap) > 0.01) {
    assistHist = []; // cambió el banco: no mezclar mediciones
    assistLastCap = capNow;
  }
  assistHist.push({ p: data.p, q: data.mc.q0, residual: data.mc.residual ?? 0, v });
  if (assistHist.length > 8) assistHist.shift();
  const n = assistHist.length;
  const p = assistHist.reduce((a, x) => a + x.p, 0) / n;
  const qLoad = assistHist.reduce((a, x) => a + x.q, 0) / n;
  const vAvg = assistHist.reduce((a, x) => a + x.v, 0) / n;

  const residual = assistHist.reduce((a, x) => a + x.residual, 0) / n;
  const fpLoad = p / Math.hypot(p, qLoad, residual);
  const sampleLabel = n === 1 ? "1 muestra" : `promedio de ${n} muestras`;
  elAssistLoad.textContent = `Carga sola (${sampleLabel}) · P ${p.toFixed(0)} W · Q ${qLoad.toFixed(0)} var · FP ${fpLoad.toFixed(2)}`;

  const combos = predictCombos(p, qLoad, vAvg, residual);
  assistCombos = {};
  let html = "";
  let note = "";
  for (const target of ASSIST_TARGETS) {
    const { combo, reaches } = pickCombo(combos, target);
    assistCombos[target] = combo;
    const chips = combo.bits
      .map((b, i) => `<span class="chip ${b ? "chip-on" : ""}">P${i + 1} ${stepCapacitances[i].toFixed(0)} µF</span>`)
      .join("");
    html += `
      <div class="assist-row ${reaches ? "ok" : "warn"}">
        <div class="assist-target">Meta ${target.toFixed(2)}</div>
        <div class="assist-main">
          <div class="assist-chips">${chips}</div>
          <div class="assist-text">${
            reaches
              ? `Conecta <strong>${comboLabel(combo.bits)}</strong> (${combo.c.toFixed(0)} µF) → FP esperado <strong>${combo.fp.toFixed(2)}</strong>`
              : `El banco no alcanza. Lo mejor es <strong>${comboLabel(combo.bits)}</strong> → FP ≈ <strong>${combo.fp.toFixed(2)}</strong>`
          }</div>
        </div>
        <button type="button" class="btn-mode assist-apply" data-target="${target}">Aplicar</button>
      </div>`;
    if (target === FP_TARGET) {
      const qLimitSq = (p * Math.tan(Math.acos(target))) ** 2 - residual ** 2;
      if (data.mc.sinusoidal_model && data.mc.residual_measured != null) {
        note = `Modelo sinusoidal · Residuo de adquisición: ${fx(data.mc.residual_measured, 0)} VA (diagnóstico).`;
      } else if (qLimitSq < 0) {
        const maxFp = p / Math.hypot(p, residual);
        note = `Con el residuo estimado de ${residual.toFixed(0)} VA, el FP máximo sería ${maxFp.toFixed(3)} aun con Q = 0.`;
      } else {
        const qLimit = Math.sqrt(qLimitSq);
        const varPerUf = capQvar(1, vAvg);
        const cMin = (qLoad - qLimit) / varPerUf;
        const cMax = (qLoad + qLimit) / varPerUf;
        note = cMax < 0 ? "La carga sola ya está en adelanto fuera de la meta."
          : `Rango teórico de capacitancia para la meta: ${Math.max(0, cMin).toFixed(1)}–${cMax.toFixed(1)} µF.`;
      }
    }
  }
  elAssistRows.innerHTML = html;
  elAssistNote.textContent = note;
}

elAssistRows.addEventListener("click", (e) => {
  const btn = e.target.closest(".assist-apply");
  if (!btn) return;
  const combo = assistCombos[btn.dataset.target];
  if (combo) applyCombo(combo.bits);
});

document.getElementById("btn-all-off").addEventListener("click", () => applyCombo([0, 0, 0]));

// ---------- Memoria de cálculo (evidencia de las fórmulas de clase) ----------
// Los números vienen del Arduino (campo "mc" del firmware). En Modo Simulación se
// calculan aquí con exactamente las mismas fórmulas (memoriaDesdeCarga).
const elMemoria = document.getElementById("memoria");
const MC_KEYS = ["s1", "fp1", "p0", "q0", "s0", "i0", "fp0", "th0", "z", "r", "x", "g", "b",
  "bc", "qc", "bn", "th", "residual", "residual_measured", "t2", "qr", "cr", "c1", "thvi", "fpvi"];
const rad2deg = (r) => (r * 180) / Math.PI;

function memoriaDesdeCarga(V, P, Q0, cUf, fpMeta = 0.95) {
  const S0 = Math.hypot(P, Q0);              // S = √(P² + Q²)
  const I0 = S0 / V;                         // I = S/V
  const th0 = Math.atan2(Q0, P);             // θ = θv − θi
  const Z = V / I0;                          // |Z| = V/I
  const R = Z * Math.cos(th0);               // R = |Z| cos θ
  const X = Z * Math.sin(th0);               // X = |Z| sen θ
  const G = R / (Z * Z);                     // Y = 1/Z = G − jB
  const B = X / (Z * Z);
  const Bc = OMEGA * cUf * 1e-6;             // Bc = ωC
  const Qc = Bc * V * V;                     // Qc = ωCV²
  const Bn = B - Bc;                         // Y' = G − j(B − ωC)
  const tan2 = Math.tan(Math.acos(fpMeta));
  const qNet = Q0 - Qc;
  const qLimit = P * tan2;
  const Qr = qNet - Math.max(-qLimit, Math.min(qLimit, qNet));
  return {
    s1: S0, fp1: P / S0, p0: P, q0: Q0, s0: S0, i0: I0, fp0: P / S0, th0: rad2deg(th0),
    z: Z, r: R, x: X, g: G * 1000, b: B * 1000, bc: Bc * 1000, qc: Qc, bn: Bn * 1000,
    th: rad2deg(Math.atan2(Bn, G)), residual: 0, t2: tan2, qr: Qr,
    cr: (Qr / (OMEGA * V * V)) * 1e6,        // C = Qc/(ωV²)
    c1: (B / OMEGA) * 1e6,                   // Q = 0: ωC = B
    thvi: rad2deg(th0), fpvi: Math.cos(th0), // FP = cos(θv − θi)
    sinusoidal_model: true, residual_measured: 0,
  };
}

const fx = (n, d = 2) => Number.isFinite(n) ? n.toFixed(d) : "—";

function renderMemoria(data) {
  const m = data.mc;
  if (!m) {
    elMemoria.innerHTML = '<p class="muted">Esperando una lectura válida de la carga…</p>';
    return;
  }
  const V = data.vrms || 0;
  const C = data.cap_uf || 0;
  const Imed = V > 0 ? m.s1 / V : 0;
  const tipo = (th) => (th >= 0 ? "atraso · inductiva" : "adelanto · capacitiva");
  const rows = [
    ["1. Medición", "P = (1/N)·Σ v·i", `<b>${fx(m.p0, 1)} W</b>`],
    ["", "S = V<sub>rms</sub>·I<sub>rms</sub>", `${fx(V, 1)} V × ${fx(Imed, 3)} A = <b>${fx(m.s1, 1)} VA</b>`],
    ["2. Carga sola", "FP = P / S", `${fx(m.p0, 1)} / ${fx(m.s0, 1)} = <b>${fx(m.fp0, 3)}</b>`],
    ["", "θ = atan2(Q, P)", `<b>${fx(m.th0, 1)}°</b> (${tipo(m.th0)})`],
    ["", m.sinusoidal_model ? "Q del modelo sinusoidal · residuo de adquisición"
      : "Q estimada · residuo no activo", `<b>${fx(m.q0, 1)} var</b> · ${fx(m.residual_measured ?? m.residual, 1)} VA`],
    ["3. Impedancia equivalente", "|Z| = V² / √(P² + Q²)", `<b>${fx(m.z || null, 2)} Ω</b>`],
    ["", "R = |Z| cos θ · X = |Z| sen θ", `R = <b>${fx(m.r, 2)} Ω</b> · X = <b>${fx(m.x, 2)} Ω</b>`],
    ["", "Factor de desplazamiento = R / |Z|", `${fx(m.r, 2)} / ${fx(m.z, 2)} = <b>${fx(m.r / m.z, 3)}</b>`],
    ["4. Admitancia", "Y = 1/Z = G − jB", `<b>${fx(m.g, 2)} − j${fx(m.b, 2)} mS</b>`],
    ["5. Capacitor en paralelo", "Z<sub>C</sub> = −j/(ωC) → B<sub>C</sub> = ωC", `ω × ${fx(C, 1)} µF = <b>${fx(m.bc, 2)} mS</b>`],
    ["", "Q<sub>C</sub> = ωC·V²", `<b>${fx(m.qc, 1)} var</b>`],
    ["6. Sistema compensado", "Y' = G − j(B − ωC)", `<b>${fx(m.g, 2)} ${m.bn >= 0 ? "−" : "+"} j${fx(Math.abs(m.bn), 2)} mS</b>`],
    ["", "S' = √(P² + Q'² + residuo²) · FP' = P / S'", `${fx(data.s, 1)} VA · FP' = <b>${fx(data.fp, 3)}</b>`],
    ["", "I' = S'/V · Q' = Q − Q<sub>C</sub>", `I' = <b>${fx(data.irms, 3)} A</b> · Q' = <b>${fx(m.q0 - m.qc, 1)} var</b>`],
    ["7. Ajuste para FP 0.95", "ΔC = ΔQ / (ωV²)", m.cr == null
      ? "Meta fuera del alcance del modelo capacitivo"
      : `<b>${fx(m.cr, 1)} µF</b> adicionales (valor negativo: retirar)`],
    ["", "Q' = 0: C = B/ω", `<b>${fx(m.c1, 1)} µF</b> totales`],
    ["8. Desplazamiento estimado", "cos(θ<sub>v</sub> − θ<sub>i</sub>)", `cos(${fx(m.thvi, 1)}°) = <b>${fx(m.fpvi, 3)}</b> · FP medido = ${fx(data.fp_measured, 3)}`],
  ];
  elMemoria.innerHTML =
    '<table class="memoria-table"><thead><tr><th>Paso</th><th>Fórmula</th><th>Sustitución y resultado</th></tr></thead><tbody>' +
    rows.map((r) => `<tr><td>${r[0]}</td><td class="formula">${r[1]}</td><td>${r[2]}</td></tr>`).join("") +
    "</tbody></table>";
}

function handleTelemetry(data) {
  const modern = typeof data.bank_simulated === "boolean";
  const source = data.demo ? "simulation" : modern ? "arduino" : "legacy";
  if (source !== lastSource) {
    stepCapacitances = source === "legacy" ? [20, 30, 80] : [60, 60, 60];
    elStepsContainer.replaceChildren();
    stepCardEls = null;
    elBankDischarge.textContent = source === "legacy" ? "Configuración anterior · 20 / 30 / 80 µF"
      : "Descarga RC teórica a 50 V (100 kΩ) · 7.33 s por paso";
    lastSource = source;
    history = [];
    assistHist = [];
  }
  const fpSystem = modern ? (data.fp_compensated ?? data.fp_measured) : data.fp;
  updateGauge(fpSystem);
  const pSystem = Number(data.p);
  const sSystem = Number(data.s);
  elFpEquation.textContent = pSystem > 0 && sSystem > 0
    ? `${fx(pSystem, 1)} W / ${fx(sSystem, 1)} VA = ${fx(pSystem / sSystem, 3)}${data.bank_simulated ? " (estimado con banco)" : ""}`
    : "Esperando lectura…";
  const qMotor = Number(data.mc?.q0);
  const thetaMotor = Number(data.mc?.th0);
  elMotorAngle.textContent = Number.isFinite(thetaMotor) && pSystem > 0 && Number.isFinite(qMotor)
    ? `atan2(${fx(qMotor, 1)} var, ${fx(pSystem, 1)} W) = ${fx(thetaMotor, 1)}° ${thetaMotor >= 0 ? "(atraso)" : "(adelanto)"}`
    : "Esperando lectura del motor…";
  elPhaseExplanation.textContent = Number.isFinite(data.phase_us)
    ? `Los ${data.phase_us} µs (${fx(data.phase_us * 360 * 60 / 1e6, 1)}°) alinean los sensores; no son el desfase eléctrico del motor.`
    : "El ajuste de fase en µs alinea los sensores; no es el desfase eléctrico del motor.";
  elFpSource.textContent = (data.demo ? "FP simulado" : data.mc?.sinusoidal_model ? "FP estimado · modelo sinusoidal"
    : data.bank_simulated ? "FP estimado con banco · muestra actual"
    : modern ? "FP medido" : "FP de firmware anterior") + " · Meta 0.950–1.000";
  if (modern && data.bank_simulated && data.fp_compensated == null && data.fp_measured != null) {
    elFpSource.textContent = "FP medido · Meta 0.950–1.000";
  }
  elFpMeasured.textContent = data.demo ? `Referencia sin banco: ${fx(data.mc?.fp0, 3)}`
    : data.bank_simulated && data.fp_compensated != null ? `FP medido: ${fx(data.fp_measured, 3)}` : "";
  elIMeasured.textContent = data.demo ? "Corriente simulada"
    : data.irms_compensated != null ? `Corriente estimada · Medida: ${fx(data.irms_measured, 2)} A`
    : modern ? "Corriente medida" : "Firmware anterior";

  elValV.textContent = `${(data.vrms || 0).toFixed(1)} V`;
  elValI.textContent = `${(data.irms || 0).toFixed(2)} A`;
  elValP.textContent = `${(data.p || 0).toFixed(1)} W`;
  elValQ.textContent = modern && !data.mc ? "—" : `${fx(data.q, 1)} var`;
  if (data.phase_us !== undefined) {
    elValPhase.textContent = `${data.phase_us} µs (${(data.phase_us * 360 * 60 / 1e6).toFixed(1)}°)`;
  }
  if (data.carga !== undefined) {
    elValCarga.textContent = data.carga === "IND" ? "Inductiva" : data.carga === "CAP" ? "Capacitiva" : "—";
  }
  if (data.cal) {
    const msg = CAL_MESSAGES[data.cal] || data.cal;
    console.log(msg);
    setTimeout(() => alert(msg), 0);
  }
  elValS.textContent = `${(data.s || 0).toFixed(1)} VA`;
  elValCap.textContent = `${(data.cap_uf || 0).toFixed(1)} µF`;
  elStatusPill.textContent = data.status || "OK";

  if (data.mode) {
    currentMode = data.mode;
    elBtnAuto.classList.toggle("active", currentMode === "AUTO");
    elBtnManual.classList.toggle("active", currentMode === "MANUAL");
  }

  renderStepCards(data.steps || [0, 0, 0], data.locked || [0, 0, 0]);
  updateAssistant(data);
  renderMemoria(data);

  history.push({ fp: fpSystem ?? 0, irms: data.irms || 0 });
  if (history.length > MAX_HISTORY) history.shift();
  drawChart();
  drawPowerTriangle(data.p || 0, data.q || 0, modern && !data.mc ? 0 : data.s || 0, fpSystem || 0);

  recordedRows.push({
    timestamp_s: ((data.t ?? Date.now()) / 1000).toFixed(2),
    load_pct: Number.isFinite(data.load_pct) ? data.load_pct : "",
    vrms: (data.vrms || 0).toFixed(2),
    irms: (data.irms || 0).toFixed(3),
    p_w: (data.p || 0).toFixed(1),
    q_var: (data.q || 0).toFixed(1),
    s_va: (data.s || 0).toFixed(1),
    fp_measured: Number.isFinite(modern ? data.fp_measured : data.fp)
      ? fx(modern ? data.fp_measured : data.fp, 4) : "",
    source,
    bank_simulated: modern ? data.bank_simulated : "",
    irms_measured: Number.isFinite(data.irms_measured) ? fx(data.irms_measured, 3) : "",
    q_estimated: Number.isFinite(data.q_estimated) ? fx(data.q_estimated, 1) : "",
    s_measured: Number.isFinite(data.s_measured) ? fx(data.s_measured, 1) : "",
    irms_compensated: Number.isFinite(data.irms_compensated) ? fx(data.irms_compensated, 3) : "",
    fp_compensated: Number.isFinite(data.fp_compensated) ? fx(data.fp_compensated, 4) : "",
    cap_uf: (data.cap_uf || 0).toFixed(1),
    steps: (data.steps || [0, 0, 0]).join(""),
    carga: data.carga || "",
    status: data.status || "",
    ...Object.fromEntries(MC_KEYS.map((k) => [`mc_${k}`, Number.isFinite(data.mc?.[k]) ? fx(data.mc[k], 4) : ""])),
  });
  elSampleCount.textContent = `${recordedRows.length} muestras`;
}

async function sendCommand(cmd) {
  if (serialWriter) {
    const encoder = new TextEncoder();
    await serialWriter.write(encoder.encode(cmd + "\n"));
  }
}

async function setStep(idx, val) {
  currentMode = "MANUAL";
  elBtnAuto.classList.remove("active");
  elBtnManual.classList.add("active");

  if (serialWriter) {
    await sendCommand(`STEP:${idx}:${val}`);
  } else if (simTimer) {
    const now = simState.t;
    if (val === 1 && now < simState.lockedUntil[idx]) return;
    if (simState.steps[idx] === val) return;
    currentMode = "MANUAL";
    simState.steps[idx] = val;
    simState.lastSwitchT = now;
    if (val === 0) simState.lockedUntil[idx] = now + lockoutSeconds(stepCapacitances[idx]);
  } else {
    alert("Conecta el Arduino por USB (o activa el Modo Simulación) para manejar los capacitores.");
  }
}
window.toggleStep = setStep;

// Aplica una combinación completa: primero desconecta lo que sobra y luego conecta lo que falta.
async function applyCombo(bits) {
  for (let i = 0; i < bits.length; i++) {
    if (!bits[i] && lastSteps[i] === 1) await setStep(i, 0);
  }
  for (let i = 0; i < bits.length; i++) {
    if (bits[i] && lastSteps[i] !== 1) await setStep(i, 1);
  }
}

elBtnAuto.addEventListener("click", async () => {
  currentMode = "AUTO";
  elBtnAuto.classList.add("active");
  elBtnManual.classList.remove("active");
  await sendCommand("MODE:AUTO");
});

elBtnManual.addEventListener("click", async () => {
  currentMode = "MANUAL";
  elBtnAuto.classList.remove("active");
  elBtnManual.classList.add("active");
  await sendCommand("MODE:MANUAL");
});

// Calibración de fase (solo con el Arduino conectado)
function requireArduino() {
  if (!serialWriter) {
    alert("Conecta el Arduino por USB para ajustar o calibrar la fase.");
    return false;
  }
  return true;
}

document.getElementById("btn-ph-minus").addEventListener("click", async () => {
  if (requireArduino()) await sendCommand("-");
});
document.getElementById("btn-ph-plus").addEventListener("click", async () => {
  if (requireArduino()) await sendCommand("+");
});
document.getElementById("btn-cal").addEventListener("click", async () => {
  if (!requireArduino()) return;
  if (confirm("¿Calibrar fase con una carga resistiva? Usa el modo MANUAL con los tres pasos apagados.")) {
    await sendCommand("CAL:1.00");
  }
});

let serialReader = null;
let keepReading = false;
let closedPromise = null;

async function closeSerialPort() {
  keepReading = false;
  if (serialReader) {
    try {
      await serialReader.cancel();
    } catch (_) {}
    serialReader = null;
  }
  if (closedPromise) {
    try {
      await closedPromise;
    } catch (_) {}
    closedPromise = null;
  }
  if (serialWriter) {
    try {
      serialWriter.releaseLock();
    } catch (_) {}
    serialWriter = null;
  }
  if (serialPort) {
    try {
      await serialPort.close();
    } catch (_) {}
    serialPort = null;
  }
  elConnBadge.textContent = "Desconectado";
  elConnBadge.className = "badge badge-off";
  elBtnConnect.textContent = "Conectar Arduino (USB)";
}

// Conexión Web Serial API (Chrome / Edge)
elBtnConnect.addEventListener("click", async () => {
  if (!("serial" in navigator)) {
    alert("Tu navegador no soporta Web Serial API. Usa Google Chrome o Microsoft Edge.");
    return;
  }

  // Si ya está conectado, hacer click sirve para desconectar limpiamente
  if (serialPort) {
    await closeSerialPort();
    return;
  }

  stopSimulation();

  try {
    serialPort = await navigator.serial.requestPort();
  } catch (err) {
    // El usuario cerró el diálogo o no seleccionó puerto
    if (err.name !== "NotFoundError") {
      console.warn("Selección de puerto cancelada:", err);
    }
    return;
  }

  try {
    await serialPort.open({ baudRate: 115200 });
    serialWriter = serialPort.writable.getWriter();

    elConnBadge.textContent = "Conectado (USB)";
    elConnBadge.className = "badge badge-on";
    elBtnConnect.textContent = "Desconectar USB";

    const decoder = new TextDecoderStream();
    closedPromise = serialPort.readable.pipeTo(decoder.writable);
    serialReader = decoder.readable.getReader();
    keepReading = true;

    let buffer = "";
    let askedJson = false;
    let gotData = false;
    const watchdog = setTimeout(() => {
      if (!gotData && serialPort) {
        elStatusPill.textContent = "SIN DATOS";
        console.warn("Sin telemetría JSON tras 5 s. Revisa baudios (115200), que el Monitor Serie esté cerrado y que el sketch powerguard_v8 esté cargado.");
      }
    }, 5000);
    while (keepReading) {
      const { value, done } = await serialReader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split("\n");
      buffer = lines.pop();
      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed) continue;
        if (trimmed.startsWith("{")) {
          let data;
          try {
            data = JSON.parse(trimmed);
          } catch (e) {
            console.warn("JSON inválido:", trimmed);
            continue;
          }
          if (data.error) {
            elStatusPill.textContent = "ERROR";
            console.error("Arduino:", data.error);
            continue;
          }
          gotData = true;
          try {
            handleTelemetry(data);
          } catch (e) {
            console.error("Error procesando telemetría:", e, data);
          }
        } else if (trimmed.startsWith("#") && !askedJson) {
          // El Arduino está en modo tabla: pedirle cambiar a JSON ("t" alterna)
          askedJson = true;
          console.warn("Arduino en modo tabla; enviando 't' para activar JSON.");
          await sendCommand("t");
        } else {
          console.log("Serial:", trimmed);
        }
      }
    }
    clearTimeout(watchdog);
  } catch (err) {
    console.error(err);
    elConnBadge.textContent = "Error / Desconectado";
    elConnBadge.className = "badge badge-off";
    if (err.name === "NetworkError" && err.message.includes("device has been lost")) {
      console.warn("Arduino desconectado físicamente.");
    } else {
      console.error("Error al abrir puerto:", err);
      if (err.name === "NetworkError") {
        alert("No se pudo abrir el puerto. Asegúrate de que el Monitor Serie de Arduino IDE esté CERRADO.");
      }
    }
  } finally {
    await closeSerialPort();
  }
});

// Detectar desconexión física del cable USB
if ("serial" in navigator) {
  navigator.serial.addEventListener("disconnect", (event) => {
    if (event.port === serialPort) {
      closeSerialPort();
    }
  });
}

// Simulador integrado para pruebas sin hardware. Se interpolan los escenarios
// de la propuesta; P y FP describen el motor antes del banco.
function simOperatingPoint(loadPct) {
  const upperIndex = SIM_MOTOR_POINTS.findIndex((point) => point.load >= loadPct);
  if (upperIndex <= 0) return { ...SIM_MOTOR_POINTS[0] };
  const upper = SIM_MOTOR_POINTS[upperIndex];
  const lower = SIM_MOTOR_POINTS[upperIndex - 1];
  const fraction = (loadPct - lower.load) / (upper.load - lower.load);
  return {
    load: loadPct,
    p: lower.p + fraction * (upper.p - lower.p),
    fp: lower.fp + fraction * (upper.fp - lower.fp),
  };
}

function renderSimScenario() {
  const { p, fp } = simOperatingPoint(simState.loadPct);
  const qLoad = p * Math.tan(Math.acos(fp));
  const iMotor = p / fp / simState.voltage;
  const { combo, reaches } = pickCombo(predictCombos(p, qLoad, simState.voltage, 0), FP_TARGET);
  elSimLoadValue.textContent = `${simState.loadPct} %`;
  elSimOperatingPoint.textContent = `Motor sin banco: ${fx(p, 1)} W · ${fx(iMotor, 2)} A a ${fx(simState.voltage, 1)} V · FP ${fx(fp, 3)} · Q ${fx(qLoad, 0)} var.`;
  elSimRecommendation.textContent = reaches
    ? `Para FP ≥ ${FP_TARGET.toFixed(2)}: ${comboLabel(combo.bits)} (${combo.c.toFixed(0)} µF) → FP estimado ${fx(combo.fp, 3)}.`
    : `Meta no alcanzable con este banco. Mejor: ${comboLabel(combo.bits)} (${combo.c.toFixed(0)} µF) → FP estimado ${fx(combo.fp, 3)}.`;
  elSimRecommendation.classList.toggle("unreachable", !reaches);
}

function runSimulationTick() {
  simState.t += 0.5;
  const { p, fp: baseFp } = simOperatingPoint(simState.loadPct);
  const qLoad = p * Math.tan(Math.acos(baseFp));
  const vrms = simState.voltage;
  let cTotal = simState.steps.reduce((acc, step, i) => acc + step * stepCapacitances[i], 0);
  let qNet = qLoad - capQvar(cTotal, vrms);

  let status = "MANUAL";
  if (currentMode === "AUTO") {
    const { combo, reaches } = pickCombo(predictCombos(p, qLoad, vrms, 0), FP_TARGET);
    if (simState.t - simState.lastSwitchT < 3) {
      status = "SETTLING";
    } else {
      const remove = simState.steps.findIndex((on, i) => on && !combo.bits[i]);
      const add = simState.steps.findIndex((on, i) => !on && combo.bits[i]);
      const step = remove >= 0 ? remove : add;
      if (step >= 0 && add === step && simState.t < simState.lockedUntil[step]) {
        status = "STEP_LOCKED";
      } else if (step >= 0) {
        const connecting = add === step;
        simState.steps[step] = Number(connecting);
        simState.lastSwitchT = simState.t;
        if (!connecting) simState.lockedUntil[step] = simState.t + lockoutSeconds(stepCapacitances[step]);
        cTotal += (connecting ? 1 : -1) * stepCapacitances[step];
        qNet = qLoad - capQvar(cTotal, vrms);
        status = connecting ? "STEP_ADDED" : "STEP_REMOVED";
      } else {
        status = reaches ? "IN_RANGE" : "TARGET_UNREACHABLE";
      }
    }
  }
  const sNet = Math.hypot(p, qNet);
  const fp = p / sNet;
  const irms = sNet / vrms;
  const lockedMs = simState.lockedUntil.map((u) => Math.max(0, Math.round((u - simState.t) * 1000)));
  handleTelemetry({
    t: Math.round(simState.t * 1000),
    vrms,
    irms,
    p,
    demo: true,
    bank_simulated: true,
    load_pct: simState.loadPct,
    fp_measured: baseFp,
    irms_measured: p / baseFp / vrms,
    s_measured: p / baseFp,
    irms_compensated: irms,
    fp_compensated: fp,
    q_estimated: qLoad,
    q: qNet, // + inductiva, - capacitiva (igual que el firmware)
    s: sNet,
    fp,
    carga: qNet >= 0 ? "IND" : "CAP",
    mc: memoriaDesdeCarga(vrms, p, qLoad, cTotal),
    steps: [...simState.steps],
    locked: lockedMs,
    cap_uf: cTotal,
    mode: currentMode,
    status,
  });
}

function stopSimulation() {
  if (simTimer) {
    clearInterval(simTimer);
    simTimer = null;
  }
  elBtnSim.textContent = "Modo Simulación";
  elBtnSimRun.textContent = "Iniciar simulación";
}

elBtnSim.addEventListener("click", () => {
  if (simTimer) {
    stopSimulation();
    elConnBadge.textContent = "Desconectado";
    elConnBadge.className = "badge badge-off";
  } else {
    // Igual que el Arduino al encender: MANUAL y sin capacitores conectados
    stepCapacitances = [60, 60, 60];
    simState.t = 0;
    simState.steps = [0, 0, 0];
    simState.lockedUntil = [0, 0, 0];
    simState.lastSwitchT = -10;
    currentMode = "MANUAL";
    simTimer = setInterval(runSimulationTick, 500);
    runSimulationTick();
    elConnBadge.textContent = "Simulación Activa";
    elConnBadge.className = "badge badge-sim";
    elBtnSim.textContent = "Detener Simulación";
    elBtnSimRun.textContent = "Detener simulación";
  }
});

elBtnSimRun.addEventListener("click", () => elBtnSim.click());
elSimLoad.addEventListener("input", () => {
  simState.loadPct = Number(elSimLoad.value);
  assistHist = [];
  renderSimScenario();
  if (simTimer) runSimulationTick();
});
elSimVoltage.addEventListener("change", () => {
  const voltage = Number(elSimVoltage.value);
  simState.voltage = Number.isFinite(voltage) ? Math.max(110, Math.min(125, voltage)) : 118;
  elSimVoltage.value = simState.voltage.toFixed(1);
  assistHist = [];
  renderSimScenario();
  if (simTimer) runSimulationTick();
});
renderSimScenario();

// Exportar registro a CSV
elBtnExport.addEventListener("click", () => {
  if (recordedRows.length === 0) {
    alert("Aún no hay mediciones registradas para exportar.");
    return;
  }
  const headers = Object.keys(recordedRows[0]);
  const csvContent = [
    headers.join(","),
    ...recordedRows.map((r) => headers.map((h) => r[h]).join(",")),
  ].join("\n");

  const blob = new Blob([csvContent], { type: "text/csv;charset=utf-8;" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `Prueba_${testCounter}.csv`;
  a.click();
  URL.revokeObjectURL(url);

  testCounter += 1;
  localStorage.setItem("powerguard_prueba_num", String(testCounter));
  updateExportButtonLabel();
});

renderStepCards();
drawChart();
drawPowerTriangle(0, 0, 0, 1);

window.addEventListener("resize", () => {
  drawChart();
  drawPowerTriangle(lastTriangleData.p, lastTriangleData.q, lastTriangleData.s, lastTriangleData.fp);
});
