# 8 · Ver el cerebro en vivo

> 🌐 [English](../brain/08-visualization.md) · **Español**

[← El piloto](07-piloto.md) · [Índice](README.md) · [Siguiente: herramientas y glosario →](09-herramientas-y-glosario.md)

---

La pestaña **1 · Brain · LiDAR map** de [`spot_brain.py`](../../../desktop/spot_brain.py)
muestra el cerebro de la mosca en 3D mientras pilota, con las neuronas que disparan
iluminadas y las sinapsis por las que pasa la señal. Dibujar 138.639 neuronas a
~30 fps junto al robot, la cueva y el mapa también requirió sus propios trucos.

---

## La forma de cada neurona

### Con morfología (recomendado)

[`download_skeletons.py`](../../../desktop/fly_brain/download_skeletons.py) descarga
los **esqueletos** de FlyWire 783 (publicados por el grupo de Jefferis, MRC LMB):
para cada neurona, un grafo de puntos y aristas que sigue su axón y sus dendritas.

Crudos son inmanejables (~1.700 vértices por neurona × 138.639 ≈ **235 millones** de
vértices), así que se simplifican en dos pasadas, agrupando vértices en una
rejilla:

| Pasada | Celda | Cuándo |
|---|---|---|
| 1 | 3 µm | Al descargar, por bloques de 2.000 neuronas (`data/skeletons/part_*.npz`) |
| 2 | 10 µm | Al juntar todo (`--merge-only`), vectorizado para todas las neuronas a la vez |

Resultado: `skeletons_lite.npz` con **~8,1 millones de segmentos**, que el visor
anima a tiempo real.

- Descarga con **48 hilos** en paralelo, con reintentos.
- **Reanudable:** cada bloque se guarda al terminar; si se corta, se sigue por el
  siguiente (~1 h y ~6 GB de tráfico en total).
- `python download_skeletons.py --limit 5000` baja una muestra para probar.

### Sin morfología

Si no se han descargado los esqueletos, el visor dibuja una **nube de puntos**, una
por neurona, en la posición de su soma. Ya da la forma del cerebro.

### Colores al estilo FlyWire

Un color por **tipo celular** (con `crc32` del nombre, para que sea estable entre
ejecuciones), con paleta según la región: cian, turquesa y amarillo en los
**lóbulos ópticos**; violeta, magenta y azul en el **cerebro central**. Fondo azul
noche, vista frontal fija que llena el panel (se gira arrastrando con el ratón).

---

## La actividad

Cada fotograma, el piloto devuelve las espigas de todas las neuronas
(`pilot.last_counts`). El visor mantiene una **actividad visible** por neurona que
se enciende con cada espiga y se apaga exponencialmente:

```python
activity *= exp(-dt_ms / 120)               # se apaga en ~120 ms
activity  = min(1, activity + 0.5 * spikes)
```

Las neuronas con actividad > 0,12 se dibujan **en naranja con toda su morfología**,
en una **capa superpuesta** (`SetLayer(1)`) que comparte la cámara del cerebro: así
se ven brillar *dentro* del cerebro aunque haya otras neuronas delante.

### Por qué hay un tope

Unas pocas neuronas de campo amplio (que se extienden por medio cerebro) bastarían
para pintarlo entero de naranja y hundir los fps. Por eso:

- Se toman las **600 más activas** como candidatas.
- Se dibujan, de más a menos activa, hasta un **presupuesto de 15.000 segmentos**.

### El truco de rendimiento: la geometría no se mueve

Los 8,1 M de puntos de la morfología se suben a la GPU **una vez**. En cada
fotograma solo cambia **qué segmentos** forman la capa activa (una lista de índices)
y su valor de color. Como los segmentos están **ordenados por neurona**, los de las
neuronas activas se calculan con aritmética vectorizada, sin bucles:

```python
counts  = sk["count"][on]
seg_idx = np.repeat(sk["start"][on] - np.cumsum(counts) + counts, counts) + np.arange(counts.sum())
```

---

## Las sinapsis

De cada neurona que acaba de disparar salen líneas a sus **3 dianas más fuertes**
del conectoma real: **naranja** si la conexión es excitatoria, **azul** si es
inhibitoria.

- Las dianas más fuertes de **todas** las neuronas se precalculan **una vez** al
  arrancar (`top_targets()`, en numba): una tabla de 138.639 × 3. En cada fotograma
  solo se indexa.
- Como mucho **220 neuronas** dibujan sus sinapsis por fotograma (las más activas).
- Las líneas salen del **centro de la morfología** de cada neurona (dentro del
  neuropilo, donde están las sinapsis de verdad), no del soma, que está en la
  corteza exterior.

---

## Las otras pestañas

| Pestaña | Arriba | Abajo |
|---|---|---|
| **2 · Eyes · Descending** | Hz de estímulo de cada grupo visual (`cmd.inputs`) | Hz de cada grupo de descendentes (`cmd.rates`) |
| **3 · History · Sensors** | Últimos 8 s de avance, giro, P9 y Giant Fiber | Gas (ppm) y temperatura (°C) |

Son una ventana directa al piloto: se ve en tiempo real qué le "enseña" el robot a la
mosca y qué contesta.

---

## Rendimiento del visor

En una RTX 4060 Laptop con un i7-12650H: **~28 ms de mediana por fotograma**, el más
lento 48 ms.

| Optimización | Por qué |
|---|---|
| **Mallas actualizadas en sitio** (`LiveLines`, `LivePoints`, `copy_from`) | Recrear actores de VTK en cada fotograma era lo que más costaba: **11,6 → 41 fps** |
| **Mapa LiDAR en otro proceso** (`ProcessPoolExecutor`) | Reconstruir la superficie congelaba el fotograma ~70 ms cada 1,5 s |
| **Solo se dibujan las pestañas visibles** | Las gráficas ocultas siguen acumulando datos, pero no se renderizan |
| **Geometría del cerebro fija en la GPU** | Solo cambian índices y colores |
| **Topes de actividad y sinapsis** | Coste acotado aunque se active medio cerebro |
| **FXAA** en vez de supersampling | El supersampling duplicaba los textos de VTK y rompía el SSAO |
| **`dt` real, acotado a 20–80 ms** | Si un fotograma tarda, el cerebro avanza ese tiempo (sin acumular retraso) pero sin saltos enormes |

Para ver en qué se va el tiempo de cada fotograma: `spot_brain.py --profile` imprime al
cerrar la mediana y el percentil 95 de cada fase (mundo + LiDAR, cerebro, marcha,
dibujo, render).

Para grabar vídeo (`--record`), el `dt` es fijo (1/24 s) y `--speed` controla
cuántos pasos de simulación entran en cada fotograma.

---

[← El piloto](07-piloto.md) · [Índice](README.md) · [Siguiente: herramientas y glosario →](09-herramientas-y-glosario.md)
