// POWERGuard - Lógica del Dashboard en Vivo (Web Serial API + Simulador)

const STEP_CAPACITANCES = [22.0, 47.0, 69.0]; // uF por paso (D12, D11, D10)
const STEP_PINS_LABEL = ["D12", "D11", "D10"];
const MAX_HISTORY = 80;

let serialPort = null;
let serialWriter = null;
let simTimer = null;
let currentMode = "AUTO";
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
const canvas = document.getElementById("live-chart");
const ctx = canvas.getContext("2d");

// Inicializar tarjetas de los 3 pasos
function renderStepCards(steps = [0, 0, 0], lockedMs = [0, 0, 0]) {
  elStepsContainer.innerHTML = "";
  for (let i = 0; i < STEP_CAPACITANCES.length; i++) {
    const isOn = steps[i] === 1;
    const lockSec = (lockedMs[i] || 0) / 1000;
    const card = document.createElement("div");
    card.className = `step-card ${isOn ? "on" : ""}`;
    card.innerHTML = `
      <div class="step-top">
        <span class="step-title">Paso ${i + 1} (${STEP_PINS_LABEL[i]})</span>
        <span class="step-state ${isOn ? "on" : "off"}">${isOn ? "CONECTADO" : "APAGADO"}</span>
      </div>
      <div class="step-cap">Capacitancia: <strong>${STEP_CAPACITANCES[i].toFixed(1)} µF</strong></div>
      <div class="step-lock">${lockSec > 0 ? `Descarga RC: ${lockSec.toFixed(1)} s` : ""}</div>
      <button class="step-btn" ${lockSec > 0 && !isOn ? "disabled" : ""} onclick="toggleStep(${i}, ${isOn ? 0 : 1})">
        ${isOn ? "Desconectar" : "Conectar"}
      </button>
    `;
    elStepsContainer.appendChild(card);
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

function drawChart() {
  const w = canvas.width;
  const h = canvas.height;
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

function handleTelemetry(data) {
  updateGauge(data.fp || 0);
  elValV.textContent = `${(data.vrms || 0).toFixed(1)} V`;
  elValI.textContent = `${(data.irms || 0).toFixed(2)} A`;
  elValP.textContent = `${(data.p || 0).toFixed(1)} W`;
  elValQ.textContent = `${(data.q || 0).toFixed(1)} var`;
  elValS.textContent = `${(data.s || 0).toFixed(1)} VA`;
  elValCap.textContent = `${(data.cap_uf || 0).toFixed(1)} µF`;
  elStatusPill.textContent = data.status || "OK";

  if (data.mode) {
    currentMode = data.mode;
    elBtnAuto.classList.toggle("active", currentMode === "AUTO");
    elBtnManual.classList.toggle("active", currentMode === "MANUAL");
  }

  renderStepCards(data.steps || [0, 0, 0], data.locked || [0, 0, 0]);

  history.push({ fp: data.fp || 0, irms: data.irms || 0 });
  if (history.length > MAX_HISTORY) history.shift();
  drawChart();

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
    status: data.status || "",
  });
  elSampleCount.textContent = `${recordedRows.length} muestras`;
}

async function sendCommand(cmd) {
  if (serialWriter) {
    const encoder = new TextEncoder();
    await serialWriter.write(encoder.encode(cmd + "\n"));
  }
}

window.toggleStep = async function (idx, val) {
  currentMode = "MANUAL";
  elBtnAuto.classList.remove("active");
  elBtnManual.classList.add("active");

  if (serialWriter) {
    await sendCommand(`STEP:${idx}:${val}`);
  } else if (simTimer) {
    const now = simState.t;
    if (val === 1 && now < simState.lockedUntil[idx]) return;
    simState.steps[idx] = val;
    if (val === 0) simState.lockedUntil[idx] = now + 6.0;
  }
};

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

// Conexión Web Serial API (Chrome / Edge)
elBtnConnect.addEventListener("click", async () => {
  if (!("serial" in navigator)) {
    alert("Tu navegador no soporta Web Serial API. Usa Google Chrome o Microsoft Edge.");
    return;
  }
  stopSimulation();
  try {
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });
    serialWriter = serialPort.writable.getWriter();

    elConnBadge.textContent = "Conectado (USB)";
    elConnBadge.className = "badge badge-on";

    const decoder = new TextDecoderStream();
    serialPort.readable.pipeTo(decoder.writable);
    const reader = decoder.readable.getReader();

    let buffer = "";
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split("\n");
      buffer = lines.pop();
      for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed.startsWith("{")) {
          try {
            handleTelemetry(JSON.parse(trimmed));
          } catch (_) {}
        }
      }
    }
  } catch (err) {
    console.error(err);
    elConnBadge.textContent = "Error / Desconectado";
    elConnBadge.className = "badge badge-off";
  }
});

// Simulador integrado para pruebas sin hardware
function runSimulationTick() {
  simState.t += 0.5;
  const loadLevels = [
    { p: 60.0, fp: 0.27 },
    { p: 245.4, fp: 0.35 },
    { p: 395.1, fp: 0.43 },
    { p: 505.9, fp: 0.51 },
    { p: 626.9, fp: 0.59 },
  ];
  const loadIdx = Math.floor(simState.t / 15) % loadLevels.length;
  const base = loadLevels[loadIdx];

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
        simState.lockedUntil[idx] = simState.t + 6.0;
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
        simState.lockedUntil[idx] = simState.t + 6.0;
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
    q: Math.abs(qNet),
    s: sNet,
    fp,
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
  a.download = `powerguard_mediciones_${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}.csv`;
  a.click();
  URL.revokeObjectURL(url);
});

renderStepCards();
drawChart();
