# POWERGuard — Compensador Automático de Factor de Potencia

**Equipo PowerGuard 12** · **Circuitos Eléctricos (PBL)** · **KEY Institute**  
**Docente:** Iván Hernandez  
**Integrantes:** Mauricio Chávez, Camila Jovel, Lucy Guerra, Fabiola Guardado  

---

## 1. Descripción del proyecto

**POWERGuard** es un sistema monofásico de **120 V / 60 Hz** diseñado para medir en tiempo real el factor de potencia (FP) verdadero de un **motor monofásico WEG de 0.5 HP** y mantenerlo automáticamente dentro del rango objetivo de **0.95 a 0.99** mediante la conexión y desconexión controlada de un **banco de capacitores de 4 pasos (J1–J4)**.

---

## 2. Estructura del repositorio

```text
sistema para PBL de CE/
├── README.md                          # Este archivo
├── docs/
│   └── arquitectura_y_cambios.md      # Detalle técnico de medición, ADS1115 y conmutación sin MOC/Triac
├── firmware/
│   └── powerguard_v8/
│       └── powerguard_v8.ino          # Firmware Arduino Nano (todo en un archivo; configuración al inicio)
├── python_model/
│   ├── requirements.txt               # Dependencias de Python
│   ├── powerguard_model.py            # Modelo teórico del motor WEG 0.5 HP y 16 combinaciones del banco
│   ├── serial_bridge.py               # Puente USB-Serial / Simulador de banco + registro CSV
│   ├── validate_measurements.py       # Comparación mediciones reales vs modelo (meta: error < 4%)
│   └── tests/
│       └── test_model.py              # Pruebas unitarias del modelo matemático y lógica de control
└── dashboard/
    ├── index.html                     # Tablero web en vivo (Web Serial API + modo demo/simulador)
    ├── styles.css                     # Estilos del tablero
    └── app.js                         # Gráficas en vivo, control manual/automático y exportación CSV
```

---

## 3. Requisitos de la rúbrica cubiertos

| Requisito del docente | Implementación en este repositorio |
| :--- | :--- |
| **Sistema monofásico 120 V, 60 Hz (motor WEG 0.5 HP)** | Parámetros de placa (0.37 kW, 9.58 A @ 110 V, FP 0.43–0.59) cargados en `powerguard_model.py`. |
| **Medición en vivo (Método 3: muestreo de $v(t)$ e $i(t)$)** | `powerguard_v8.ino` muestrea `ZMPT101B` y `SCT-013` vía `ADS1115` (I2C) y calcula $V_{rms}, I_{rms}, P, Q, S, FP$ verdadero. |
| **Corrección de retraso del ADS1115** | Interpolación lineal temporal entre muestras consecutivas de tensión/corriente + calibración de fase por software (comando `CAL`, guardada en EEPROM). |
| **Modelo en Python con fórmulas de clase** | `powerguard_model.py` calcula $S, P, Q, FP, Q_C, C$, descarga $V(t)=V_0 e^{-t/RC}$, error % y predice las 16 combinaciones del banco. |
| **Comparación modelo vs mediciones (< 4% error)** | `validate_measurements.py` evalúa lecturas CSV contra el modelo teórico y reporta el cumplimiento de la meta `< 4%`. |
| **Control del banco (0.95–0.99) y seguridad** | Histéresis 0.95–0.99, detección de sobrecompensación, tiempo entre maniobras y bloqueo por descarga RC. |
| **Visualización en vivo** | `dashboard/index.html` con lectura directa por USB (Web Serial API en Chrome/Edge) o vía simulador Python. |

---

## 4. Guía rápida de uso

### A. Modelo en Python y pruebas unitarias
```bash
cd python_model
python powerguard_model.py
python -m unittest discover -s tests -v
```

### B. Simulador / Puente Serial y validación
Para probar el sistema completo sin tener el Arduino conectado (genera telemetría realista del motor WEG y simula la respuesta del banco de capacitores):
```bash
cd python_model
python serial_bridge.py --simulate --log pruebas_simuladas.csv
```
Para validar mediciones reales exportadas desde el dashboard o el puente serial:
```bash
python validate_measurements.py --input pruebas_simuladas.csv
```

### C. Firmware en Arduino Nano
1. Abrir `firmware/powerguard_v8/powerguard_v8.ino` en **Arduino IDE**.
2. No necesita librerías externas (el ADS1115 se lee por I2C por software).
3. Revisar la sección CONFIGURACIÓN al inicio del `.ino` (pines, capacitancias 20/30/80 µF, calibración de `ZMPT101B` y `SCT-013`).
4. Subir al **Arduino Nano** (ATmega328P, 115200 baud). Ver el [manual de uso](docs/MANUAL_DE_USO.md).

### D. Dashboard HTML en vivo
1. Abrir `dashboard/index.html` en **Google Chrome** o **Microsoft Edge**.
2. Hacer clic en **Conectar Arduino (USB)** para usar Web Serial API directamente, o en **Modo Simulación** para pruebas sin hardware.
