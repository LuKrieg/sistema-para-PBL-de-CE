# Arquitectura del Sistema y Registro de Cambios

## 1. Medición del factor de potencia

El ZMPT101B mide la tensión en A0 del Arduino Nano. El SCT-013-000 mide la corriente a través del ADS1115 en modo diferencial continuo A0–A1. La comunicación I²C por software usa A1 y A2 del Nano.

El firmware interpola la tensión en un búfer circular para emparejarla con la corriente. El retardo inicial es 1700 µs, ajustable con `PH:<µs>` o con `+` y `-`. El comando `CAL:1.00` calibra la fase y la polaridad del SCT con carga resistiva en modo manual y con los tres pasos apagados. El ajuste se guarda en EEPROM.

- La potencia activa $P$ es la covarianza de las muestras de tensión y corriente alineadas, con sus escalas calibradas.
- La potencia aparente es $S=V_{rms}I_{rms}$ y el FP verdadero es $P/S$. El coseno del ángulo estimado se presenta como factor de desplazamiento.
- Con tensión casi sinusoidal se estima $Q$ a partir de la covarianza de la corriente con la tensión retrasada un cuarto de ciclo nominal. $Q>0$ indica atraso y $Q<0$, adelanto.
- El modelo separa el residuo no activo $D=\sqrt{\max(0,S^2-P^2-Q^2)}$. El residuo puede incluir distorsión y error de adquisición. Su valor por sí solo no cuantifica la distorsión armónica del motor.

## 2. Banco simulado y control

La propuesta original contemplaba cuatro pasos de conmutación. En el prototipo actual D12, D11 y D10 controlan tres LEDs que representan tres pasos de 60 µF. Los ocho estados corresponden a 0, 60, 120 y 180 µF.

El banco virtual usa un equivalente sinusoidal a 60 Hz. Obtiene la magnitud de $Q$ como $\sqrt{S^2-P^2}$ y usa el signo de la estimación en cuadratura. El banco ideal aporta $Q_C=2\pi fCV^2$ y la predicción usa $Q'=Q-Q_C$, $S'=\sqrt{P^2+Q'^2}$, $I'=S'/V$ y $FP'=P/S'$. La magnitud de Q en cuadratura y el residuo de adquisición se conservan como diagnósticos. La impedancia equivalente describe los terminales a partir de $P+jQ$ de ese modelo.

El control mantiene el estado con $FP'\ge 0.95$, incluso si hay un adelanto leve. Fuera de la banda, evalúa una etapa en el sentido de corrección de $Q'$. Solo cambia si alcanza la meta o mejora el FP por al menos 0.005. Espera 3 s entre maniobras y simula un bloqueo de reconexión de 7.33 s por paso, calculado mediante $RC\ln(169.7/50)$ con 100 kΩ. En AUTO apaga los pasos si desaparece la señal de carga.

Los escenarios de Python mantienen $P$ y $Q$ de la referencia al variar la tensión, mientras que $Q_C$ cambia con $V^2$. En vacío, 118 V, 8.10 A y un FP supuesto de 0.20 dan $P=191.16$ W. Con señales sinusoidales se obtienen aproximadamente 936.5 var en atraso. Con 165 µF el modelo da FP 0.938 y con 180 µF da FP 0.999 en adelanto. A 25 % de carga y 118 V, los valores de la propuesta dan FP 0.6485 con 180 µF y 0.9941 con 120 µF. A 50 % y 120 V, el mejor estado disponible da 0.9368. Estos resultados dependen de los escenarios de entrada.

El modelo supone 0 W de potencia mecánica útil en vacío. En los demás escenarios usa una fracción de los 367.75 W nominales como salida ilustrativa y calcula la eficiencia como $P_{out}/P_{in}$. Una evaluación experimental de eficiencia necesita la potencia útil del eje, por ejemplo a partir de torque y velocidad.

## 3. Protocolo USB Serial

El Nano envía un objeto JSON por línea a 115200 baudios. La trama incluye la tensión `vrms`, la potencia activa `p` y los siguientes campos:

| Campos | Significado |
|---|---|
| `fp_measured`, `irms_measured`, `s_measured` | FP, corriente y potencia aparente de la adquisición |
| `q_estimated` | Reactiva estimada por cuadratura de la adquisición |
| `fp_compensated`, `irms_compensated` | Predicción con el banco simulado |
| `bank_simulated`, `steps`, `cap_uf`, `locked` | Tipo de banco, estado de etapas, capacitancia y bloqueo restante |
| `fp`, `irms`, `s`, `q` | Magnitudes del sistema para lectores anteriores |
| `mc` | Memoria de cálculo, incluido el residuo y el ajuste de capacitancia respecto del estado actual |

Las magnitudes sin una lectura válida se envían como `null` en los campos explícitos de FP y Q. El dashboard y el puente Serial identifican el origen en el CSV. Los archivos anteriores siguen siendo legibles como datos históricos. Para el banco simulado, el validador compara el FP medido con el escenario de carga sola y presenta aparte el cumplimiento estimado con el banco.

Comandos de uso habitual:

- `MODE:AUTO` y `MODE:MANUAL`
- `STEP:<índice>:<0|1>` con índices de 0 a 2. El comando pasa a manual y respeta el bloqueo.
- `CAL:1.00` calibra la fase bajo las condiciones de la sección 1.
- `m` imprime la memoria de cálculo y `t` alterna JSON y texto.
