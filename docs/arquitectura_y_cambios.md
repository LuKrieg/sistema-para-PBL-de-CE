# Arquitectura del Sistema y Registro de Cambios

## 1. Cadena de medición (True Power Factor)

El sistema implementa el **Método 3 (Muestreo de tensión y corriente)** descrito en la rúbrica de la materia:

1. **Sensor de tensión (`ZMPT101B`)**:
   - Aísla la red de 120 V mediante transformador.
   - Entrega una señal senoidal centrada en $V_{CC}/2 \approx 2.5\,\text{V}$.
   - Conectado al canal `A0` del `ADS1115` (entrada analógica 16 bits, I2C `0x48`).

2. **Sensor de corriente (`SCT-013-030` / respaldo `SCT-013-000`)**:
   - `SCT-013-030`: relación `30 A : 1 V` con resistencia de carga (*burden*) integrada.
   - `SCT-013-000` (respaldo): relación `100 A : 50 mA` con resistencia externa de $\sim 33\,\Omega$.
   - Señal centrada a $V_{CC}/2$ con divisor resistivo ($2 \times 10\,\text{k}\Omega$) y capacitor de $10\,\mu\text{F}$, conectada al canal `A1` del `ADS1115` (o diferencial `A2-A3` según configuración en `config.h`).
   - Ubicación física: mide la corriente total (motor + banco de capacitores en paralelo) para obtener el FP real de la instalación.

3. **Corrección de desfase por multiplexado del ADS1115**:
   - El `ADS1115` posee un único ADC interno multiplexado entre canales. A `860 SPS` (modo continuo/disparo rápido), leer `A0` y luego `A1` introduce un retraso $\Delta t_{adc} \approx 1.16\,\text{ms}$ ($\approx 25^\circ$ a $60\,\text{Hz}$), lo cual distorsionaría severamente el cálculo de $P$ y $FP$ si no se compensa.
   - **Solución implementada en el firmware**:
     - Se muestrea de forma alternada con marca de tiempo (`micros()`): $v_0, i_0, v_1, i_1, \dots$
     - Se realiza **interpolación lineal temporal** de $v(t)$ en el instante exacto de cada muestra de corriente $i_k$:
       $$v_{interp}(t_{i,k}) = v_k + \frac{t_{i,k} - t_{v,k}}{t_{v,k+1} - t_{v,k}} (v_{k+1} - v_k)$$
     - Adicionalmente, se incluye un parámetro `PHASE_CALIBRATION_FACTOR` en `config.h` para compensar el pequeño error de fase propio del transformador de corriente (`SCT-013`) y del `ZMPT101B` ($\approx 0.33^\circ$) durante la calibración contra un medidor de referencia.

4. **Cálculo de magnitudes eléctricas**:
   - Remoción dinámica de componente DC ($\bar{v}, \bar{i}$) sobre un número entero de ciclos de $60\,\text{Hz}$ (p. ej., 6 ciclos = $100\,\text{ms}$):
     $$v_k' = v_k - \bar{v}, \qquad i_k' = i_k - \bar{i}$$
   - Valores eficaces y potencia activa:
     $$V_{rms} = \sqrt{\frac{1}{N}\sum_{k=1}^{N} (v_k')^2}, \qquad I_{rms} = \sqrt{\frac{1}{N}\sum_{k=1}^{N} (i_k')^2}, \qquad P = \frac{1}{N}\sum_{k=1}^{N} v_k' i_k'$$
   - Potencia aparente, reactiva y factor de potencia verdadero:
     $$S = V_{rms} \cdot I_{rms}, \qquad Q = \sqrt{\max(0, S^2 - P^2)}, \qquad FP = \frac{P}{S}$$

---

## 2. Cambio de diseño: eliminación de MOC3031 y Triacs

En la propuesta de la Semana 14 se planteó conmutar los 4 pasos (`J1` a `J4`) con optotriacs `MOC3031` y triacs `BTA16` / `BTA26-H`.  
**Modificación aplicada:** se retiraron los `MOC3031` y los triacs, pasando a **conmutación por relés** controlados por los pines digitales `D4`, `D5`, `D6` y `D7` del Arduino Nano.

### Implicaciones técnicas cubiertas en el código
- **Aislamiento**: los módulos de relés para Arduino incorporan optoacopladores en la entrada de control y aislamiento galvánico en los contactos de potencia (`120 V AC`).
- **Polaridad de disparo**: configurable mediante `RELAY_ACTIVE_LOW` en `config.h` (`true` para módulos de relés estándar activos en `LOW`; `false` si se usan drivers activos en `HIGH`).
- **Protección contra picos y rebotes**:
  - Se mantiene un tiempo mínimo entre maniobras (`MIN_SWITCH_INTERVAL_MS = 3000 ms`).
  - Se prohíbe reconectar cualquier paso antes de que se descargue a través de su resistencia de $100\,\text{k}\Omega$ (`DISCHARGE_LOCKOUT_MS` por paso, calculado con $5\tau = 5 R C$).
- **Detección de sobrecompensación**:
  - Como $FP = P / (V_{rms} I_{rms})$ no indica si la corriente está atrasada o adelantada, al conectar un paso cuando $FP < 0.95$, el controlador verifica en la siguiente ventana de medición que el $FP$ haya aumentado. Si el $FP$ disminuyó (señal de que se cruzó la unidad hacia carga capacitiva/adelantada), el controlador revierte ese paso inmediatamente.

---

## 3. Protocolo de comunicación USB Serial (Nano $\leftrightarrow$ Dashboard / Python)

El Arduino Nano envía tramas **JSON por línea** a `115200 baud` cada $\sim 500\,\text{ms}$:

```json
{"t":12500,"vrms":120.4,"irms":7.82,"p":542.1,"q":765.3,"s":941.5,"fp":0.576,"steps":[1,1,0,0],"cap_uf":34.5,"mode":"AUTO","locked":[0,0,4200,0],"status":"COMPENSATING"}
```

Y acepta comandos simples por Serial (desde el Dashboard HTML o Python):
- `MODE:AUTO` / `MODE:MANUAL`
- `STEP:<idx>:<0|1>` (ej. `STEP:0:1` para encender el paso 1 en modo manual, respetando el bloqueo de descarga)
- `STATUS` (solicita lectura inmediata)
