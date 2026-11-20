# 1 · El conectoma

[← Índice](README.md) · [Siguiente: el modelo LIF →](02-modelo-lif.md)

---

## Qué es un conectoma

Un conectoma es el **mapa completo de qué neurona se conecta con cuál** en un
sistema nervioso. El de la mosca adulta lo terminó en 2024 el consorcio
**FlyWire** (Dorkenwald et al., *Nature* 634): cortaron un cerebro en miles de
láminas, las fotografiaron al microscopio electrónico y reconstruyeron cada neurona
y cada sinapsis, con ayuda de redes neuronales y de miles de correcciones humanas.

El resultado (versión **783**) es lo que simulamos:

- **138.639 neuronas**, cada una con un identificador único (`root_id`).
- **~15 millones de pares conectados** (pre → post), y para cada par cuántas
  sinapsis hay.
- Para cada neurona, su **neurotransmisor** predicho, y con él el **signo**:
  acetilcolina = excitatoria (+), GABA y glutamato = inhibitorias (−).
- **Anotaciones** (Schlegel et al. 2024): tipo celular (`LPLC2`, `DNp01`…), clase
  (`visual_projection`, `descending`…), hemisferio (`left`/`right`) y posición del
  soma.

> No hay ningún parámetro "aprendido" ni entrenado. Los pesos son literalmente
> *número de sinapsis × signo*, medidos en el tejido.

---

## Los archivos

`download_data.py` baja tres archivos públicos a `desktop/fly_brain/data/` (fuera
del repositorio, ~130 MB):

| Archivo | Origen | Contenido |
|---|---|---|
| `Connectivity_783.parquet` | Shiu et al. 2024 (MIT) | Una fila por par: `Presynaptic_Index`, `Postsynaptic_Index`, `Excitatory x Connectivity` (sinapsis con signo) |
| `Completeness_783.csv` | Shiu et al. 2024 (MIT) | La lista ordenada de `root_id`: la fila *i* es la neurona *i* del simulador |
| `neuron_annotations.tsv` | Schlegel et al. 2024 (CC BY 4.0) | Tipo, clase, lado, neurotransmisor y posición de cada neurona |

---

## De la tabla a una matriz dispersa

`Connectome.load()` (en [`connectome.py`](../../../desktop/fly_brain/connectome.py))
convierte la tabla en una **matriz de pesos W** de 138.639 × 138.639:

- **Filas = neurona postsináptica**, **columnas = presináptica**.
- `W[post, pre]` = sinapsis con signo de `pre` a `post`.
- Se guarda en formato **CSC** (*Compressed Sparse Column*).

### ¿Por qué dispersa?

Una matriz densa de 138.639² en `float32` ocuparía **~77 GB**. Pero de esos 19.200
millones de huecos solo ~15 millones tienen algo: una ocupación del **0,08 %**. En
formato disperso solo se guardan los no-ceros:

| Array | Tipo | Tamaño aproximado |
|---|---|---|
| `data` (peso) | `float32` | 15 M × 4 B ≈ 60 MB |
| `indices` (fila = post) | `int32` | 15 M × 4 B ≈ 60 MB |
| `indptr` (dónde empieza cada columna) | `int64` | 138.640 × 8 B ≈ 1,1 MB |

Unos **120 MB** en total: cabe de sobra en la memoria de cualquier GPU moderna.

### ¿Por qué CSC y no CSR?

Porque el simulador pregunta siempre lo mismo: *"la neurona `pre` acaba de
disparar, ¿a quién le llega?"*. En CSC, **todas las dianas de una neurona están
contiguas en memoria**:

```
for k in range(indptr[pre], indptr[pre + 1]):
    entrada[indices[k]] += data[k]
```

Leer una columna es recorrer un trozo continuo de `indices` y `data`, que es lo
más rápido que puede hacer una CPU o una GPU. Es la base de la
[propagación por eventos](05-reduccion-del-tiempo.md#1-propagación-dirigida-por-eventos).

### `sum_duplicates()` y la caché

- Si un par pre→post aparece varias veces, `sum_duplicates()` lo une en un solo
  peso.
- La primera vez se guarda el resultado en `data/connectome_cache.npz`. Las
  siguientes se carga directamente (tres arrays de NumPy) sin volver a leer el
  parquet ni reconstruir la matriz.

---

## Buscar neuronas por nombre

```python
from connectome import Connectome
c = Connectome.load()

c.find('DNp01')                # Giant Fiber, ambos lados -> índices del simulador
c.find('LC10a', side='left')   # solo el hemisferio izquierdo
c.find_prefix('LC10')          # LC10a, LC10b, LC10c-1, ...
```

`find()` busca en `cell_type` y también en `hemibrain_type` (el nombre que tenía
la neurona en el conectoma parcial anterior, *hemibrain*), porque la literatura usa
ambos.

> **Convención de lados.** El hemisferio izquierdo del cerebro procesa el ojo
> izquierdo, así que `side='left'` en FlyWire = **lado izquierdo del robot**.

---

## Las neuronas que importan para el robot

De las 138.639, el piloto toca directamente unas pocas familias:

| Familia | Tipo | Qué hacen en la mosca real |
|---|---|---|
| **Entrada** | `LC10a/c/d` | Proyecciones visuales que detectan un objeto pequeño en movimiento (cortejo, persecución) |
| | `LC9`, `LC31a` | Proyecciones visuales que se asocian a caminar hacia delante |
| | `LPLC2`, `LC4` | Detectores de *looming*: algo que crece rápido en la retina (un depredador que se acerca) |
| **Salida** | `DNp01` (**Giant Fiber**) | La neurona más gruesa: dispara el salto de escape |
| | `DNp09` (**P9**) | Caminar hacia delante |
| | `DNa02` | Girar hacia su lado |
| | `MDN` (*moonwalker*) | Caminar hacia atrás |

Las **descendentes** (*DN*) son las neuronas que bajan del cerebro al cordón
nervioso ventral, que es lo que mueve las patas de la mosca. Son el punto natural
donde "enchufar" el cuerpo del robot. Cómo se eligieron estas familias se cuenta
en [el piloto](07-piloto.md) y en [herramientas](09-herramientas-y-glosario.md#el-barrido-de-673-tipos).

---

[← Índice](README.md) · [Siguiente: el modelo LIF →](02-modelo-lif.md)
