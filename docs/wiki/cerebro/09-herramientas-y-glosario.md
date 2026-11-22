# 9 · Herramientas, pruebas y glosario

> 🌐 [English](../brain/09-tools-and-glossary.md) · **Español**

[← Ver el cerebro en vivo](08-visualizacion.md) · [Índice](README.md)

---

## `probe.py`: experimentar con el cerebro

[`probe.py`](../../../desktop/fly_brain/probe.py) estimula un tipo celular desde el
reposo y muestra qué **descendentes motoras** responden. Es la herramienta con la que
se decidió qué neuronas usa el piloto.

```bash
python probe.py LPLC2 LC4              # looming en ambos ojos -> Giant Fiber
python probe.py LC10a --side left      # objeto a la izquierda -> DNa02 izquierda
python probe.py LC9 --hz 100 --ms 300  # frecuencia y duración a medida
python probe.py --device cpu --dt 0.5  # forzar CPU y paso grueso
python probe.py --scan                 # todos los tipos sensoriales -> data/scan.csv
```

Salida típica:

```
GPU · dt = 0.1 ms
Estimulando N neuronas ['LPLC2', 'LC4']  a 150 Hz durante 500 ms

Descendentes motoras conocidas:
   DNp01  left    ...  Hz  ##############   Giant Fiber: escape (salto)
   DNp01 right    ...  Hz  ##############   Giant Fiber: escape (salto)
   ...
Top 12 descendentes:
   ...
```

(Las cifras exactas dependen del estímulo y de la semilla.)

Las descendentes que vigila, con su función conocida en la literatura (Namiki 2018,
Bidaye 2014/2020, Cande 2018…):

| Tipo | Función |
|---|---|
| DNp01 | Giant Fiber: escape (salto) |
| DNp09 | P9: caminar hacia delante |
| DNa02 | Giro hacia su lado |
| DNa01 | Giro / dirección |
| MDN | *Moonwalker*: caminar hacia atrás |
| DNp02, DNp04, DNp11 | Respuesta a *looming* |
| DNg11 | Aseo de las patas delanteras |

Sin GPU usa `dt = 0,5 ms` para ir a tiempo real (< 5 % de diferencia en estas
pruebas).

---

## El barrido de 673 tipos

`probe.py --scan` estimula, **uno por uno y por lado**, todos los tipos sensoriales y
de proyección visual con al menos 2 neuronas (673 estímulos, 150 Hz, 300 ms cada
uno) y guarda la respuesta de todas las descendentes motoras en `data/scan.csv`.

Son 673 × 300 ms ≈ **200 s de vida de mosca**, que en la GPU se simulan en **~45 s**
(gracias a que `reset()` reutiliza el mismo cerebro y el mismo CUDA Graph).

Lo que salió:

| Estímulo | Descendente | Conducta |
|---|---|---|
| LPLC2, LC4, LC6 | DNp01 (Giant Fiber), 110–170 Hz | Escape ante algo que se acerca |
| LPLC2 + LC4 de un lado | DNp02/04/11 del mismo lado | Respuesta lateralizada |
| LC10a / c-2 / d de un lado | DNa02 del mismo lado, 77–133 Hz | Perseguir un objeto |
| LC9, LC31a | DNp09 (P9), 50–92 Hz | Caminar hacia delante |
| LPC1, LC9, LC6 | MDN, 15–32 Hz | Caminar hacia atrás |

De esta tabla sale directamente la elección de entradas y salidas del
[piloto](07-piloto.md).

---

## `benchmark.py`: medir el rendimiento

[`benchmark.py`](../../../desktop/fly_brain/benchmark.py) mide las cifras de
[cómo reducimos el tiempo](05-reduccion-del-tiempo.md):

| Medida | Qué saca |
|---|---|
| `speed` | Veces tiempo real y µs por paso, en CPU y GPU, con `dt` 0,1 y 0,5 ms |
| `activity` | Espigas por paso y sinapsis entregadas por paso: el ahorro **real** de la propagación por eventos |
| `graphs` | GPU con y sin CUDA Graphs |
| `branches` | CPU: integración sin ramas frente a la versión con `if` |

```bash
python benchmark.py                   # todo
python benchmark.py --only activity   # una medida
python benchmark.py --markdown        # tablas para pegar en la wiki
python benchmark.py --synthetic       # red aleatoria, sin datos (solo para probar el script)
```

Guarda los resultados en `data/benchmark.json`.

## `figures.py`: las gráficas de la wiki

[`figures.py`](../../../desktop/fly_brain/figures.py) genera las imágenes de
`docs/wiki/assets/`, en español y en inglés:

| Figura | Comando | Necesita |
|---|---|---|
| Velocidad de cada versión | `python figures.py speed` | Nada |
| Original contra nuestro, neurona a neurona | `python figures.py scatter` | Haber corrido `validate_brian2.py` |
| Espigas durante un escape (*raster*) | `python figures.py raster` | El conectoma de FlyWire |
| Una neurona LIF recibiendo espigas | `python figures.py neuron` | Nada |

## Todas las pruebas

```bash
cd desktop/fly_brain
.venv\Scripts\python -m unittest discover -s tests -v     # 31: simulador (CPU y GPU), piloto y mundo
cd ..
fly_brain\.venv\Scripts\python -m unittest discover -s tests -v   # 17: cinemática y marcha
```

- **Simulador:** el LIF en CPU y GPU, con redes pequeñas hechas a mano (ver
  [validación](06-validacion.md#tests-automáticos)).
- **Piloto:** el paso de integración según el hardware (0,1 ms en GPU, 0,5 ms en CPU) y
  que el comando quede acotado.
- **Mundo:** geometría de la cueva, LiDAR, mapa sin duplicados, gas, calor y esquivar
  paredes; en la cueva real, la ruta, el LiDAR y el gas.

---

## Preguntas frecuentes

**¿La mosca "aprende" a explorar la cueva?**
No. No hay entrenamiento ni plasticidad: los pesos son el conectoma medido y no
cambian. La conducta sale de cómo está cableado el cerebro real; el piloto solo
elige por dónde entra la información y qué se lee.

**¿Es "el cerebro de una mosca" de verdad?**
Es el cableado completo y real de un cerebro de mosca, con un modelo de neurona muy
simplificado (LIF, sin dendritas, sin neuromoduladores, sin adaptación). Ese modelo
reproduce conductas medidas en moscas vivas (Shiu et al. 2024), pero no es una
simulación biofísica completa.

**¿Pilota el robot físico?**
Todavía no: solo el Spot Micro simulado. El robot físico se maneja desde la GUI y no
lleva sensores de misión, que no se integraron a propósito para poder adaptarlo a
distintos proyectos. Ver [del simulador al robot real](10-del-simulador-al-robot-real.md).

**¿Necesito GPU?**
No es obligatorio: todo funciona en CPU. Sin GPU el piloto usa `dt = 0,5 ms`, con el
que la CPU va a ~1× (el visor, con el dibujo encima, algo por debajo). Con una GPU
NVIDIA y `cupy-cuda12x[ctk]` usa el `dt` del artículo (0,1 ms) y va 8 veces por
delante del tiempo real.

**¿Por qué no usar Brian2, NEST o GeNN directamente?**
Brian2 es el original y necesita compilar C++ en cada experimento; además está
pensado para corridas completas, no para avanzar 40 ms y devolver el control.
Un simulador propio de ~400 líneas, validado contra el original, se integra
directamente en el bucle del robot y no añade dependencias pesadas.

**¿Por qué el gas entra por los ojos?**
Porque las vías olfativa y térmica del modelo saturan el cerebro y no dan dirección.
Ver [el piloto](07-piloto.md#por-qué-entrar-por-los-ojos).

---

## Glosario

| Término | Significado |
|---|---|
| **Conectoma** | Mapa completo de las conexiones entre neuronas de un sistema nervioso |
| **FlyWire** | Consorcio que reconstruyó el conectoma completo del cerebro de *Drosophila* adulta (v783) |
| **LIF** | *Leaky Integrate-and-Fire*: neurona modelada como un condensador con fugas que dispara al superar un umbral |
| **Espiga** | Potencial de acción: el "disparo" de una neurona |
| **Periodo refractario** | Tiempo tras una espiga en que la neurona no puede volver a disparar |
| **Retardo sináptico** | Tiempo que tarda una espiga en llegar a su diana (1,8 ms aquí) |
| **Poisson** | Proceso aleatorio de eventos independientes a una tasa media; aquí, la estimulación |
| **Optogenética** | Técnica que activa neuronas concretas con luz; el modelo la imita con entradas Poisson |
| **Descendente (DN)** | Neurona que baja del cerebro al cordón nervioso y controla el cuerpo |
| **Giant Fiber (DNp01)** | La descendente que dispara el salto de escape de la mosca |
| **P9 (DNp09)** | Descendente asociada a caminar hacia delante |
| **MDN** | *Moonwalker descending neuron*: caminar hacia atrás |
| **LC, LPLC** | *Lobula columnar*, *lobula plate–lobula columnar*: proyecciones visuales del lóbulo óptico al cerebro central |
| **Looming** | Estímulo visual de algo que crece rápido en la retina: se está acercando |
| **Sustitución sensorial** | Presentar una información por un sentido distinto del original (aquí, gas y calor como escenas visuales) |
| **CSC** | *Compressed Sparse Column*: formato de matriz dispersa con cada columna contigua en memoria |
| **Propagación por eventos** | Entregar solo la salida de las neuronas que dispararon, en vez de multiplicar toda la matriz |
| **Kernel** | Función que se ejecuta en la GPU, en miles de hilos a la vez |
| **Warp** | Grupo de 32 hilos de GPU que ejecutan la misma instrucción a la vez |
| **`atomicAdd`** | Suma en memoria que no se pierde aunque muchos hilos escriban a la vez en la misma dirección |
| **CUDA Graph** | Secuencia de lanzamientos de kernels grabada una vez y reproducida con una sola llamada |
| **SIMD** | *Single Instruction, Multiple Data*: una instrucción de CPU que opera sobre varios números a la vez |
| **Subnormal** | Número de coma flotante extremadamente pequeño, muy lento de operar en muchas CPU |
| **numba** | Compilador JIT que convierte funciones de Python en código máquina |
| **CuPy** | Biblioteca tipo NumPy para GPU NVIDIA, que también compila kernels CUDA escritos en C |
| **Brian2** | Simulador de redes neuronales en Python, usado por el modelo original |

---

## Referencias

- Dorkenwald et al. 2024. *Neuronal wiring diagram of an adult brain.* **Nature** 634.
- Schlegel et al. 2024. *Whole-brain annotation and multi-connectome cell typing of
  Drosophila.* **Nature** 634.
- Shiu et al. 2024. *A Drosophila computational brain model reveals sensorimotor
  processing.* **Nature**. Código: [philshiu/Drosophila_brain_model](https://github.com/philshiu/Drosophila_brain_model) (MIT).
- Anotaciones: [flyconnectome/flywire_annotations](https://github.com/flyconnectome/flywire_annotations) (CC BY 4.0).
- Esqueletos: FlyWire 783, grupo de Jefferis (MRC LMB), CC BY 4.0.

---

[← Ver el cerebro en vivo](08-visualizacion.md) · [Índice](README.md) · [Siguiente: del simulador al robot real →](10-del-simulador-al-robot-real.md)
