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
3. Al arrancar, el sistema está en **MANUAL** y los 3 capacitores apagados.

## 4. Calibrar la fase (una vez)
Con el motor encendido y **todos los capacitores apagados**, escribir en la casilla el FP real conocido (0.20 motor en vacío; 1.00 con un foco o plancha) y pulsar **Calibrar**. Repetir si el FP no queda exacto. Se guarda en el Arduino. Los botones `−` / `+` ajustan la fase fina.

## 5. Corregir el factor de potencia
- Los botones **Conectar/Desconectar** de cada paso (20, 30 y 80 µF) encienden los LEDs. El FP sube al conectar capacitores. Meta: 0.95–0.99.
- Los LEDs **simulan** los capacitores: el firmware resta Qc = ωCV² a la Q medida y muestra Q, S, I y FP ya compensados.
- El **Asistente de compensación** calcula, con la carga medida, qué pasos conectar para 0.85 y 0.95. *Aplicar* los enciende. Si el banco no alcanza, lo indica y dice cuántos µF faltan.
- Un capacitor recién desconectado se bloquea hasta descargarse a 50 V (2.4 s, 3.7 s y 9.8 s).
- Si el FP baja al conectar más capacitores, la carga ya pasó a **capacitiva** (se sobrecompensó).

## 6. Evidencia de los cálculos
- Tarjeta **Memoria de cálculo**: cada fórmula de clase (S, FP, Q, |Z|, R, X, Y = G − jB, Qc, FP', C requerida) con los números en vivo.
- **Exportar Prueba_N.csv** guarda todas las lecturas, incluidas las columnas `mc_*`.
- Por el Monitor Serie (115200), el comando `m` imprime la memoria de cálculo; `t` alterna JSON/texto.

## 7. Problemas comunes
| Síntoma | Solución |
|---|---|
| "Access is denied" al subir | Desconectar el dashboard y cerrar el Monitor Serie |
| Dashboard sin datos / `SIN DATOS` | Cerrar el Monitor Serie, comprobar 115200 baudios y que el sketch cargado sea el v8 |
| Capacitancia activa ≠ 130 µF con los 3 pasos | El Arduino tiene un sketch viejo: volver a subir v8 |
| Faltan los botones de calibración | Caché: Ctrl+F5 |
