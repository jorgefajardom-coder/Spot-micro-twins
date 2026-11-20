# 3 · El simulador en CPU (`lif.py`)

[← El modelo LIF](02-modelo-lif.md) · [Índice](README.md) · [Siguiente: el simulador en GPU →](04-simulador-gpu.md)

---

## Por qué reescribirlo

El modelo original de Shiu et al. está escrito con **Brian2**, un simulador
excelente para investigación, pero que para este proyecto tenía tres problemas:

1. **Necesita un compilador de C++.** Brian2 genera código C++ y lo compila. En
   Windows eso obliga a tener Visual Studio y a pelearse con su detección (lo que
   hace `msvc_env.py`, que ahora solo se usa para la validación).
2. **Está pensado para correr un experimento entero** (`run(1 s)`) y devolver las
   espigas al final, no para avanzar **40 ms, devolver el control al robot, cambiar
   el estímulo y seguir**, 24 veces por segundo.
3. **Es lento para este uso:** ver [la comparación](05-reduccion-del-tiempo.md#el-punto-de-partida).

Así que [`lif.py`](../../../desktop/fly_brain/lif.py) reimplementa el mismo modelo
en Python + **numba** (compila Python a código máquina con LLVM, sin compilador de
C++ externo).

---

## La API

```python
from connectome import Connectome
from lif import Params, make_brain

c = Connectome.load()
brain = make_brain(c, Params(dt=0.1))      # GPU si hay CUDA; si no, CPU
brain.stimulate((c.find('LPLC2'), 150),    # pares (índices, Hz)
                (c.find('LC4'), 150))
counts = brain.run(100)                    # espigas por neurona en 100 ms
brain.rate(counts, 100, c.find('DNp01'))   # Hz medios de la Giant Fiber
brain.reset()                              # vuelve al reposo, mismo objeto
```

- `stimulate()` **sustituye** la estimulación anterior (sin argumentos, la apaga).
- `run(ms)` avanza y devuelve **cuántas espigas** dio cada neurona en ese tramo.
- El estado (`v`, `g`, refractarios, buffer de retardo) **persiste** entre llamadas:
  el cerebro sigue "vivo" de un fotograma al siguiente.

`make_brain()` elige solo: si `lif_gpu.available()` devuelve una `BrainGPU`; si no,
una `Brain` de CPU. Las dos tienen la misma API y pasan los mismos tests.

---

## Estructura de datos

Todo son arrays planos de NumPy, uno por variable (*structure of arrays*):

| Array | Tipo | Por qué ese tipo |
|---|---|---|
| `v`, `g` | `float32` | Precisión de sobra para mV; la mitad de memoria que `float64` |
| `ref_left` | `int16` | Pasos de refractario que quedan (máx. 21) |
| `no_refractory` | `bool` | Las neuronas estimuladas no tienen refractario |
| `pending` | `float32[18, n]` | Buffer circular del retardo sináptico |
| `w_data` | `float32` | Pesos **ya multiplicados por `w_syn`** (una multiplicación menos por sinapsis) |
| `w_indices`, `w_indptr` | `int32`, `int64` | La matriz CSC |
| `spike_count` | `int64` | Contador acumulado de espigas |

---

## El paso, en dos fases

### Fase 1 · `_integrate`: todas las neuronas, sin ramas

```python
for i in range(n):
    act = ref_left[i] == 0
    vn  = v_0 + a_vv * (v[i] - v_0) + a_vg * g[i]
    gn  = g[i] * a_gg
    gn  = gn if abs(gn) >= eps else 0.0
    fired[i] = act and vn > v_th
    v[i] = vn if act else v[i]
    g[i] = gn + row[i] if act else g[i]
    row[i] = 0.0
    ref_left[i] = ref_left[i] - 1 if ref_left[i] > 0 else 0
```

Fíjate en lo que **no** hay: ningún `if ... continue` por neurona. Se calcula todo
para todas y luego se **selecciona** con expresiones condicionales, que el
compilador convierte en instrucciones *select* / *blend*. Así LLVM puede
**vectorizar** el bucle con SIMD (AVX2: 8 neuronas por instrucción en `float32`).

> Con la versión "natural" (saltarse las refractarias con un `if`) era **~7 veces
> más lento**: qué neuronas están refractarias es aleatorio, el predictor de saltos
> de la CPU falla constantemente y además el bucle deja de vectorizarse.
> Ver [optimización 4](05-reduccion-del-tiempo.md#4-bucle-sin-ramas-simd-en-cpu).

### Fase 2 · `_run`: estímulos, reset y propagación

```python
for j in range(len(stim_idx)):                 # solo las estimuladas
    if random() < stim_p[j]:
        v[stim_idx[j]] += w_poi
for pre in range(n):
    if fired[pre]:                             # casi siempre False
        v[pre] = v_rst; g[pre] = 0
        if not no_refractory[pre]:
            ref_left[pre] = ref_steps
        counts[pre] += 1
        for k in range(indptr[pre], indptr[pre+1]):   # solo su columna
            row[indices[k]] += data[k]
```

Aquí sí hay una rama, pero es **muy predecible**: en cada paso dispara una fracción
minúscula de las neuronas, así que el predictor acierta "no dispara" casi siempre.
Y cuando una dispara, solo se recorre **su columna** del conectoma: es la
[propagación por eventos](05-reduccion-del-tiempo.md#1-propagación-dirigida-por-eventos),
la optimización más importante de todas.

---

## Otros detalles que suman

- **`@njit(cache=True)`**: el código compilado se guarda en disco; la segunda
  ejecución no recompila (ahorra segundos al arrancar).
- **`fastmath=True`** en `_integrate`: permite a LLVM reordenar operaciones de coma
  flotante para vectorizar mejor.
- **`G_EPS = 1e-4` mV**: cuando `g` decae por debajo de eso se pone a 0. Si no, tras
  muchos pasos de decaimiento exponencial `g` llega a números **subnormales**
  (< 1,2·10⁻³⁸), que en x86 son decenas de veces más lentos de operar. 10⁻⁴ mV es
  despreciable frente a los 7 mV hasta el umbral.
- **Un solo bucle de Python por llamada:** `run(ms)` entra en numba una vez y hace
  todos los pasos dentro; Python no ve cada paso.
- **Semilla reproducible:** `_seed()` siembra el generador de numba (que es
  distinto del de NumPy) para que dos ejecuciones con la misma semilla den las
  mismas espigas.

---

## Rendimiento

En un i7-12650H, con *looming* (LPLC2 + LC4 a 150 Hz):

| `dt` | Velocidad | Coste por paso |
|---|---|---|
| 0,1 ms (original) | **0,29×** tiempo real | ~345 µs |
| 0,5 ms | **0,99×** tiempo real | — (5× menos pasos) |

Con `dt = 0,5 ms` la CPU va prácticamente a tiempo real y las frecuencias cambian
solo un **3 %**. Por eso `probe.py` usa 0,5 ms cuando no hay GPU. Para ir 8 veces
*más* rápido que el tiempo real con el `dt` del artículo hace falta la GPU →
[siguiente página](04-simulador-gpu.md).

---

[← El modelo LIF](02-modelo-lif.md) · [Índice](README.md) · [Siguiente: el simulador en GPU →](04-simulador-gpu.md)
