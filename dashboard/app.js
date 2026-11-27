// POWERGuard - Lógica del Dashboard en Vivo (Web Serial API + Simulador)

const STEP_CAPACITANCES = [20.0, 30.0, 80.0]; // uF por paso (D12, D11, D10)
const STEP_PINS_LABEL = ["D12", "D11", "D10"];
const MAX_HISTORY = 80;

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
  CAL_VALOR_INVALIDO: "No se pudo calibrar: el FP de referencia debe estar entre 0.05 y 1.00.",
};
let history = [];
let recordedRows = [];

// Estado de simulación local
const simState = {
  t: 0,
  steps: [0, 0, 0],
  lockedUntil: [0, 0, 0],
  lastSwitchT: -10,
  awaitingCheck: false,
  lastAdded: -1,
  fpBefore: 0,
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
const elCalFp = document.getElementById("cal-fp");

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
  for (let i = 0; i < STEP_CAPACITANCES.length; i++) {
    const card = document.createElement("div");
    card.className = "step-card";
    card.innerHTML = `
      <div class="step-top">
        <span class="step-title">Paso ${i + 1} (${STEP_PINS_LABEL[i]})</span>
        <span class="step-state off">APAGADO</span>
      </div>
      <div class="step-cap">Capacitancia: <strong>${STEP_CAPACITANCES[i].toFixed(1)} µF</strong></div>
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
    el.lock.textContent = lockSec > 0 && !isOn ? `Descargando capacitor: ${lockSec.toFixed(1)} s` : "";
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

  if (clamped >= 0.95 && clamped <= 0.99) {
    elGaugeArc.setAttribute("stroke", "#10b981");
  } else if (clamped >= 0.85) {
    elGaugeArc.setAttribute("stroke", "#38bdf8");
  } else {
    elGaugeArc.setAttribute("stroke", "#f59e0b");
  }
  elValFp.textContent = clamped.toFixed(3);
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

  // Líneas de meta FP (0.95 y 0.99)
  const y095 = padT + plotH * (1 - 0.95);
  const y099 = padT + plotH * (1 - 0.99);
  ctx.save();
  ctx.setLineDash([5, 5]);
  ctx.strokeStyle = "#10b981";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, y095); ctx.lineTo(w - padR, y095);
  ctx.moveTo(padL, y099); ctx.lineTo(w - padR, y099);
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

  const phiDeg = Math.acos(Math.max(0, Math.min(1, fp))) * (180 / Math.PI);
  elTriangleAngle.textContent = `φ = ${phiDeg.toFixed(1)}°`;

  const padL = 28, padR = 84, padT = 26, padB = 34;
  const maxW = Math.max(40, w - padL - padR);
  const maxH = Math.max(40, h - padT - padB);

  if (s < 1.0) {
    triCtx.fillStyle = "#9ca3af";
    triCtx.font = "13px Segoe UI, sans-serif";
    triCtx.fillText("Esperando carga...", w / 2 - 55, h / 2);
    return;
  }

  const maxDim = Math.max(p, Math.abs(q), 1);
  const scale = Math.min(maxW / maxDim, maxH / maxDim);
  const px = Math.max(20, p * scale);
  const qy = Math.max(4, Math.abs(q) * scale);

  const x0 = padL;
  const y0 = h - padB;
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

  // Hipotenusa S (Aparente)
  triCtx.strokeStyle = fp >= 0.95 && fp <= 0.99 ? "#10b981" : "#e5e7eb";
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

  triCtx.fillStyle = fp >= 0.95 && fp <= 0.99 ? "#10b981" : "#e5e7eb";
  triCtx.fillText(`S = ${s.toFixed(0)} VA`, x0 + px * 0.25, y0 - qy * 0.55 - 6);
}

// ---------- Asistente de compensación ----------
// Con P y Q medidos y los capacitores que hay conectados, reconstruye la Q de la carga sola
// (Q_carga = Q_medida + Qc_conectado) y predice el FP de cada combinación del banco.
const OMEGA = 2 * Math.PI * 60;
const ASSIST_TARGETS = [0.85, 0.95];
const elAssistLoad = document.getElementById("assist-load");
const elAssistRows = document.getElementById("assist-rows");
const elAssistNote = document.getElementById("assist-note");
let assistHist = [];
let assistLastCap = -1;
let assistCombos = {};

function capQvar(cUf, v) {
  return OMEGA * cUf * 1e-6 * v * v;
}

function predictCombos(p, qLoad, v) {
  const out = [];
  for (let m = 0; m < 8; m++) {
    const bits = [m & 1, (m >> 1) & 1, (m >> 2) & 1];
    const c = bits.reduce((a, b, i) => a + b * STEP_CAPACITANCES[i], 0);
    const qNet = qLoad - capQvar(c, v);
    const fp = p / Math.hypot(p, qNet);
    out.push({ bits, c, qNet, fp, ind: qNet >= 0 });
  }
  return out;
}

function pickCombo(combos, target) {
  const ok = combos.filter((x) => x.ind && x.fp >= target);
  if (ok.length) {
    ok.sort((a, b) => Math.abs(a.fp - (target + 0.02)) - Math.abs(b.fp - (target + 0.02)));
    return { combo: ok[0], reaches: true };
  }
  const ind = combos.filter((x) => x.ind);
  ind.sort((a, b) => b.fp - a.fp);
  return { combo: ind[0] || combos[0], reaches: false };
}

function comboLabel(bits) {
  const names = bits.map((b, i) => (b ? `Paso ${i + 1}` : null)).filter(Boolean);
  return names.length ? names.join(" + ") : "ningún capacitor";
}

function updateAssistant(data) {
  const v = data.vrms || 0;
  const s = data.s || 0;
  if (s < 1 || v < 50) {
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
  const qLoadNow = (data.q || 0) + capQvar(capNow, v);
  assistHist.push({ p: data.p || 0, q: qLoadNow, v });
  if (assistHist.length > 8) assistHist.shift();
  const n = assistHist.length;
  const p = assistHist.reduce((a, x) => a + x.p, 0) / n;
  const qLoad = assistHist.reduce((a, x) => a + x.q, 0) / n;
  const vAvg = assistHist.reduce((a, x) => a + x.v, 0) / n;

  const fpLoad = p / Math.hypot(p, qLoad);
  elAssistLoad.textContent = `Carga sola: P ${p.toFixed(0)} W · Q ${qLoad.toFixed(0)} var · FP ${fpLoad.toFixed(2)}`;

  const combos = predictCombos(p, qLoad, vAvg);
  const bankMax = STEP_CAPACITANCES.reduce((a, b) => a + b, 0);
  assistCombos = {};
  let html = "";
  let note = "";
  for (const target of ASSIST_TARGETS) {
    const { combo, reaches } = pickCombo(combos, target);
    assistCombos[target] = combo;
    const cNeed = (qLoad - p * Math.tan(Math.acos(target))) / (OMEGA * vAvg * vAvg) * 1e6;
    const chips = combo.bits
      .map((b, i) => `<span class="chip ${b ? "chip-on" : ""}">P${i + 1} ${STEP_CAPACITANCES[i].toFixed(0)} µF</span>`)
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
    if (!reaches && target === 0.95) {
      note = `Para llegar a 0.95 con esta carga hacen falta ≈ ${cNeed.toFixed(0)} µF y el banco suma ${bankMax.toFixed(0)} µF` +
        ` (faltan ≈ ${Math.max(0, cNeed - bankMax).toFixed(0)} µF).`;
    } else if (reaches && target === 0.95) {
      note = `Capacitancia teórica para 0.95: ≈ ${cNeed.toFixed(0)} µF.`;
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
// Los números vienen del Arduino (campo "mc" del firmware v7). En Modo Simulación se
// calculan aquí con exactamente las mismas fórmulas (memoriaDesdeCarga).
const elMemoria = document.getElementById("memoria");
const MC_KEYS = ["s1", "fp1", "p0", "q0", "s0", "i0", "fp0", "th0", "z", "r", "x", "g", "b",
  "bc", "qc", "bn", "th", "t1", "t2", "qr", "cr", "c1", "thvi", "fpvi"];
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
  const tan1 = Q0 / P;
  const tan2 = Math.tan(Math.acos(fpMeta));
  const Qr = P * (tan1 - tan2);              // Qc = P(tan θ1 − tan θ2)
  return {
    s1: S0, fp1: P / S0, p0: P, q0: Q0, s0: S0, i0: I0, fp0: P / S0, th0: rad2deg(th0),
    z: Z, r: R, x: X, g: G * 1000, b: B * 1000, bc: Bc * 1000, qc: Qc, bn: Bn * 1000,
    th: rad2deg(Math.atan2(Bn, G)), t1: tan1, t2: tan2, qr: Qr,
    cr: (Qr / (OMEGA * V * V)) * 1e6,        // C = Qc/(ωV²)
    c1: (B / OMEGA) * 1e6,                   // FP = 1: ωC = B
    thvi: rad2deg(th0), fpvi: Math.cos(th0), // FP = cos(θv − θi)
  };
}

const fx = (n, d = 2) => Number(n).toFixed(d);

function renderMemoria(data) {
  const m = data.mc;
  if (!m) {
    elMemoria.innerHTML = '<p class="muted">Esperando carga… (enciende el motor; requiere firmware v7)</p>';
    return;
  }
  const V = data.vrms || 0;
  const C = data.cap_uf || 0;
  const Imed = V > 0 ? m.s1 / V : 0;
  const Yn = Math.hypot(m.g, m.bn);
  const In = (V * Yn) / 1000;
  const tipo = (th) => (th >= 0 ? "atraso · inductiva" : "adelanto · capacitiva");
  const rows = [
    ["1. Medición", "P = (1/N)·Σ v·i", `<b>${fx(m.p0, 1)} W</b>`],
    ["", "S = V<sub>rms</sub>·I<sub>rms</sub>", `${fx(V, 1)} V × ${fx(Imed, 3)} A = <b>${fx(m.s1, 1)} VA</b>`],
    ["2. Triángulo de potencias", "FP = P / S = cos θ", `${fx(m.p0, 1)} / ${fx(m.s0, 1)} = <b>${fx(m.fp0, 3)}</b>`],
    ["", "θ = θ<sub>v</sub> − θ<sub>i</sub> = arccos(FP)", `<b>${fx(m.th0, 1)}°</b> (${tipo(m.th0)})`],
    ["", "Q = S·sen θ", `${fx(m.s0, 1)} × sen(${fx(m.th0, 1)}°) = <b>${fx(m.q0, 1)} var</b>`],
    ["3. Triángulo de impedancia", "|Z| = V / I", `${fx(V, 1)} / ${fx(m.i0, 3)} = <b>${fx(m.z, 2)} Ω</b>`],
    ["", "R = |Z| cos θ · X = |Z| sen θ", `R = <b>${fx(m.r, 2)} Ω</b> · X = <b>${fx(m.x, 2)} Ω</b>`],
    ["", "FP = R / |Z|", `${fx(m.r, 2)} / ${fx(m.z, 2)} = <b>${fx(m.r / m.z, 3)}</b>`],
    ["4. Admitancia", "Y = 1/Z = G − jB", `<b>${fx(m.g, 2)} − j${fx(m.b, 2)} mS</b>`],
    ["5. Capacitor en paralelo", "Z<sub>C</sub> = −j/(ωC) → B<sub>C</sub> = ωC", `377 × ${fx(C, 1)} µF = <b>${fx(m.bc, 2)} mS</b>`],
    ["", "Q<sub>C</sub> = ωC·V²", `<b>${fx(m.qc, 1)} var</b>`],
    ["6. Sistema compensado", "Y' = G − j(B − ωC)", `<b>${fx(m.g, 2)} ${m.bn >= 0 ? "−" : "+"} j${fx(Math.abs(m.bn), 2)} mS</b>`],
    ["", "FP' = G / |Y'| = cos θ'", `${fx(m.g, 2)} / ${fx(Yn, 2)} = <b>${fx(m.g / Yn, 3)}</b> (θ' = ${fx(m.th, 1)}°, ${tipo(m.th)})`],
    ["", "I' = V·|Y'| · Q' = Q − Q<sub>C</sub>", `I' = <b>${fx(In, 3)} A</b> · Q' = ${fx(m.q0, 1)} − ${fx(m.qc, 1)} = <b>${fx(m.q0 - m.qc, 1)} var</b>`],
    ["7. Corrección requerida", "Q<sub>C</sub> = P (tan θ<sub>1</sub> − tan θ<sub>2</sub>)", `${fx(m.p0, 1)} × (${fx(m.t1, 3)} − ${fx(m.t2, 3)}) = <b>${fx(m.qr, 1)} var</b>`],
    ["", "C = Q<sub>C</sub> / (ωV²)", `<b>${fx(m.cr, 1)} µF</b> para FP 0.95 · conectado: ${fx(C, 1)} µF`],
    ["", "FP = 1: ωC = B → C = B/ω", `<b>${fx(m.c1, 1)} µF</b>`],
    ["8. Verificación", "FP = cos(θ<sub>v</sub> − θ<sub>i</sub>)", `cos(${fx(m.thvi, 1)}°) = <b>${fx(m.fpvi, 3)}</b> (vs P/S = ${fx(m.fp1, 3)})`],
  ];
  elMemoria.innerHTML =
    '<table class="memoria-table"><thead><tr><th>Paso</th><th>Fórmula</th><th>Sustitución y resultado</th></tr></thead><tbody>' +
    rows.map((r) => `<tr><td>${r[0]}</td><td class="formula">${r[1]}</td><td>${r[2]}</td></tr>`).join("") +
    "</tbody></table>";
}

function handleTelemetry(data) {
  updateGauge(data.fp || 0);
  elValV.textContent = `${(data.vrms || 0).toFixed(1)} V`;
  elValI.textContent = `${(data.irms || 0).toFixed(2)} A`;
  elValP.textContent = `${(data.p || 0).toFixed(1)} W`;
  elValQ.textContent = `${(data.q || 0).toFixed(1)} var`;
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

  history.push({ fp: data.fp || 0, irms: data.irms || 0 });
  if (history.length > MAX_HISTORY) history.shift();
  drawChart();
  drawPowerTriangle(data.p || 0, data.q || 0, data.s || 0, data.fp || 0);

  recordedRows.push({
    timestamp_s: ((data.t || Date.now()) / 1000).toFixed(2),
    vrms: (data.vrms || 0).toFixed(2),
    irms: (data.irms || 0).toFixed(3),
    p_w: (data.p || 0).toFixed(1),
    q_var: (data.q || 0).toFixed(1),
    s_va: (data.s || 0).toFixed(1),
    fp_measured: (data.fp || 0).toFixed(4),
    cap_uf: (data.cap_uf || 0).toFixed(1),
    steps: (data.steps || [0, 0, 0]).join(""),
    carga: data.carga || "",
    status: data.status || "",
    ...Object.fromEntries(MC_KEYS.map((k) => [`mc_${k}`, data.mc && data.mc[k] !== undefined ? Number(data.mc[k]).toFixed(4) : ""])),
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
    simState.steps[idx] = val;
    if (val === 0) simState.lockedUntil[idx] = now + lockoutSeconds(STEP_CAPACITANCES[idx]);
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
  const fpRef = parseFloat(elCalFp.value);
  if (!(fpRef > 0 && fpRef <= 1)) {
    alert("Escribe el FP real de la carga (entre 0.05 y 1.00).");
    return;
  }
  const ok = confirm(
    `¿Calibrar la fase con FP real = ${fpRef.toFixed(2)}?\n\n` +
    "La carga debe estar encendida y SIN capacitores conectados " +
    "(motor en vacío → 0.20, foco o plancha → 1.00).\n" +
    "Si el FP medido aún no queda exacto, pulsa Calibrar otra vez."
  );
  if (ok) await sendCommand(`CAL:${fpRef.toFixed(3)}`);
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

// Simulador integrado para pruebas sin hardware
function runSimulationTick() {
  simState.t += 0.5;
  // Motor WEG 0.5 HP en vacío: P ≈ 60 W y FP ≈ 0.20 sin capacitores (Q ≈ 294 var, I ≈ 2.5 A)
  const base = { p: 60.0, fp: 0.2 };

  const vrms = 120.0 + (Math.random() - 0.5) * 0.8;
  const cTotal = simState.steps.reduce((acc, s, i) => acc + s * STEP_CAPACITANCES[i], 0);
  const qLoad = base.p * Math.tan(Math.acos(base.fp));
  const qCap = 2 * Math.PI * 60 * (cTotal * 1e-6) * vrms * vrms;
  const qNet = qLoad - qCap;
  const sNet = Math.hypot(base.p, qNet);
  const fp = Math.min(1.0, base.p / sNet);
  const irms = sNet / vrms;

  let status = currentMode === "MANUAL" ? "MANUAL" : "IN_RANGE";
  if (currentMode === "AUTO" && simState.t - simState.lastSwitchT >= 3.0) {
    if (simState.awaitingCheck && simState.lastAdded >= 0) {
      simState.awaitingCheck = false;
      if (fp < simState.fpBefore) {
        const idx = simState.lastAdded;
        simState.steps[idx] = 0;
        simState.lockedUntil[idx] = simState.t + lockoutSeconds(STEP_CAPACITANCES[idx]);
        simState.lastSwitchT = simState.t;
        status = "OVERCOMP_REVERT";
      }
      simState.lastAdded = -1;
    } else if (fp < 0.95) {
      const avail = [0, 1, 2].filter((i) => simState.steps[i] === 0 && simState.t >= simState.lockedUntil[i]);
      if (avail.length > 0) {
        avail.sort((a, b) => STEP_CAPACITANCES[a] - STEP_CAPACITANCES[b]);
        const idx = avail[0];
        simState.fpBefore = fp;
        simState.steps[idx] = 1;
        simState.lastAdded = idx;
        simState.awaitingCheck = true;
        simState.lastSwitchT = simState.t;
        status = "STEP_ADDED";
      }
    } else if (fp > 0.99) {
      const active = [0, 1, 2].filter((i) => simState.steps[i] === 1);
      if (active.length > 0) {
        active.sort((a, b) => STEP_CAPACITANCES[a] - STEP_CAPACITANCES[b]);
        const idx = active[0];
        simState.steps[idx] = 0;
        simState.lockedUntil[idx] = simState.t + lockoutSeconds(STEP_CAPACITANCES[idx]);
        simState.lastSwitchT = simState.t;
        status = "STEP_REMOVED";
      }
    }
  }

  const lockedMs = simState.lockedUntil.map((u) => Math.max(0, Math.round((u - simState.t) * 1000)));
  handleTelemetry({
    t: Math.round(simState.t * 1000),
    vrms,
    irms,
    p: base.p,
    q: qNet, // + inductiva, - capacitiva (igual que el firmware)
    s: sNet,
    fp,
    carga: qNet >= 0 ? "IND" : "CAP",
    mc: memoriaDesdeCarga(vrms, base.p, qLoad, cTotal),
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
}

elBtnSim.addEventListener("click", () => {
  if (simTimer) {
    stopSimulation();
    elConnBadge.textContent = "Desconectado";
    elConnBadge.className = "badge badge-off";
    elBtnSim.textContent = "Modo Simulación";
  } else {
    // Igual que el Arduino al encender: MANUAL y sin capacitores conectados
    simState.t = 0;
    simState.steps = [0, 0, 0];
    simState.lockedUntil = [0, 0, 0];
    simState.lastSwitchT = -10;
    simState.awaitingCheck = false;
    simState.lastAdded = -1;
    currentMode = "MANUAL";
    simTimer = setInterval(runSimulationTick, 500);
    elConnBadge.textContent = "Simulación Activa";
    elConnBadge.className = "badge badge-sim";
    elBtnSim.textContent = "Detener Simulación";
  }
});

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
