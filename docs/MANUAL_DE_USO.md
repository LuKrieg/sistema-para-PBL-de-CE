# Manual de uso rápido · POWERGuard (v8)

## 1. Qué se necesita
- Arduino Nano con el circuito armado: ADS1115 (SDA→A1, SCL→A2), ZMPT101B → A0, pinza SCT-013 en el ADS1115, LEDs en D12, D11 y D10.
- Arduino IDE y un navegador **Chrome o Edge** (Firefox y Safari no sirven).
- Archivos: `firmware/powerguard_v8/powerguard_v8.ino` y la carpeta `dashboard/` completa (`index.html`, `app.js`, `styles.css`).

> **Seguridad:** el lado de red del ZMPT y el motor trabajan a 120 V. Conecta o cambia el cableado con el breaker abierto, aísla los bornes de red y pon un fusible pequeño en la línea del ZMPT. El ZMPT va en paralelo con el motor (después del breaker) y la pinza abraza **un solo** cable.

## 2. Cargar el código
1. Abrir `powerguard_v8.ino` en Arduino IDE (es un solo archivo).
2. Elegir **Arduino Nano** y el puerto COM. Si falla la subida, probar Processor → *ATmega328P (Old Bootloader)*.
3. **Upload**. Antes, cerrar el Monitor Serie y pulsar *Desconectar USB* en el dashboard (si no, "Access is denied").

## 3. Abrir el dashboard
1. Abrir `dashboard/index.html` en Chrome o Edge (recargar con **Ctrl+F5** si hubo cambios).
2. **Conectar Arduino (USB)** y elegir el puerto. *Modo Simulación* sirve para probar sin hardware.
3. Al arrancar, el sistema está en **MANUAL** y los tres pasos simulados apagados.

## 4. Calibrar la fase (una vez)
Con una carga resistiva conectada, selecciona **MANUAL**, apaga los tres pasos y pulsa **Calibrar fase** (comando `CAL:1.00`). El ajuste queda guardado en el Arduino. Los botones `−` / `+` permiten ajustar el retardo en incrementos de 50 µs. La calibración de fase y la comprobación de las escalas de tensión y corriente son operaciones independientes.

## 5. Corregir el factor de potencia
- Los botones **Conectar/Desconectar** encienden los LEDs de las tres etapas simuladas de 60 µF. El tablero muestra el FP y la corriente estimados con el banco junto a sus lecturas del sensor.
- En **AUTO**, el sistema mantiene cualquier FP entre 0.95 y 1.00. Por debajo de la meta evalúa agregar capacitancia si $Q>0$ o retirarla si $Q<0$. El controlador espera 3 s entre maniobras y exige que el siguiente paso mejore el FP.
- El **Asistente de compensación** evalúa las combinaciones para 0.85 y 0.95 e indica si el banco puede llegar a la meta. *Aplicar* selecciona los pasos propuestos.
- Cada paso simula un bloqueo de reconexión de 7.33 s basado en el modelo RC con 100 kΩ y 60 µF.

## 6. Evidencia de los cálculos
- Tarjeta **Memoria de cálculo**: cada fórmula de clase (S, FP, Q, |Z|, R, X, Y = G − jB, Qc, FP', C requerida) con los números en vivo.
- **Exportar Prueba_N.csv** guarda el origen de cada lectura, `fp_measured` y `irms_measured` del Arduino, `fp_compensated` y `irms_compensated` del banco simulado, y la memoria en las columnas `mc_*`.
- Por el Monitor Serie (115200), el comando `m` imprime la memoria de cálculo; `t` alterna JSON/texto.

## 7. Problemas comunes
| Síntoma | Solución |
|---|---|
| "Access is denied" al subir | Desconectar el dashboard y cerrar el Monitor Serie |
| Dashboard sin datos / `SIN DATOS` | Cerrar el Monitor Serie, comprobar 115200 baudios y que el sketch cargado sea el v8 |
| Capacitancia activa de 130 µF con los tres pasos | El Arduino tiene la configuración anterior. Volver a cargar el sketch del repositorio. |
| Faltan los botones de calibración | Caché: Ctrl+F5 |
