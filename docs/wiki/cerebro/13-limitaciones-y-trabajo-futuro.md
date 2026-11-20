# 13 · Limitaciones y trabajo futuro

> 🌐 [English](../brain/13-limitations-and-future-work.md) · **Español**

[← Solución de problemas](12-solucion-de-problemas.md) · [Índice](README.md)

---

## Limitaciones conocidas

### Del modelo de neurona

| Limitación | Consecuencia |
|---|---|
| **LIF puntual**: sin dendritas, sin canales iónicos detallados | Reproduce frecuencias y conductas medidas, pero no la biofísica fina |
| **Sin adaptación** | Una neurona estimulada sigue disparando igual indefinidamente; los circuitos muy recurrentes se desbocan |
| **Sin neuromoduladores** (dopamina, octopamina…) | No hay estados internos: hambre, sueño, excitación |
| **Sin plasticidad** | La mosca no aprende; los pesos son siempre los del conectoma |
| **Signo por neurotransmisor predicho** | Algunas neuronas pueden tener el signo equivocado (el glutamato, por ejemplo, puede ser excitatorio en algunos receptores) |
| **Solo el cerebro** | No incluye el cordón nervioso ventral; por eso las descendentes se leen directamente y `gait.py` hace de médula |

### De la integración con el robot

- **Olfato y calor no sirven tal cual:** saturan el cerebro y no dan dirección (ver
  [el piloto](07-piloto.md#por-qué-entrar-por-los-ojos)). Se usa sustitución sensorial
  por circuitos visuales.
- **El robot físico no lleva sensores de misión:** gas, temperatura ambiente y LiDAR
  **no se integraron a propósito**, para que la plataforma se pueda adaptar a
  distintos proyectos. En la simulación son sensores virtuales. Ver
  [del simulador al robot real](10-del-simulador-al-robot-real.md).
- **El cerebro solo pilota la simulación:** el robot físico se maneja desde la GUI.
- **El brazo no manipula:** solo orienta la cabeza y el LiDAR, y no comprueba choques
  con el techo.
- **El mundo sabe dónde está el robot:** la simulación no necesita estimar su posición;
  un robot real sí.

### De rendimiento

- **Sin GPU NVIDIA**, el cerebro corre en CPU con `dt = 0,5 ms` (~1× en la simulación
  sola). Con el dibujo encima, el visor se queda algo por debajo del tiempo real.
- **Solo CUDA:** no hay versión para GPU AMD o Apple (CuPy tiene soporte experimental
  de ROCm, sin probar aquí).
- **Algunas cifras de la wiki no están medidas todavía en el PC de desarrollo** con
  `benchmark.py` (espigas por paso, efecto exacto de los CUDA Graphs y de quitar las
  ramas). Ver [medirlo tú mismo](05-reduccion-del-tiempo.md#medirlo-tú-mismo).

---

## Trabajo futuro

### Cerebro

- **Mejorar el lóbulo antenal** (inhibición lateral, adaptación) para usar el olfato de
  verdad en vez de la sustitución sensorial.
- **Adaptación por neurona** (una corriente lenta que sube con cada espiga): el modelo
  sería más estable y permitiría estímulos continuos más largos.
- **Conectar el cordón nervioso ventral** (conectomas MANC / FANC) para que las patas
  salgan también de neuronas reales, en vez de `gait.py`.

### Rendimiento

- **Pesos en `float16`** o índices de 24 bits: menos tráfico en la propagación.
- **Reordenar las neuronas** (por región o con Cuthill–McKee) para que las dianas de
  cada columna queden más juntas en memoria.
- **Varios ensayos a la vez** en la misma GPU para acelerar `probe.py --scan` y la
  validación.

### Robot

- **Integrar los sensores de cada proyecto** en el robot físico y escribir el adaptador
  de sentidos ([plan](10-del-simulador-al-robot-real.md#cómo-se-conectaría)).
- **Localización** con odometría + IMU, o SLAM con el LiDAR.
- **Brazo real** con pinza que manipule objetos.

---

[← Solución de problemas](12-solucion-de-problemas.md) · [Índice](README.md)
