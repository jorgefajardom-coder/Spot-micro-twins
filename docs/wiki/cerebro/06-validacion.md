# 6 · Validación contra el modelo original

> 🌐 [English](../brain/06-validation.md) · **Español**

[← Cómo reducimos el tiempo](05-reduccion-del-tiempo.md) · [Índice](README.md) · [Siguiente: el piloto →](07-piloto.md)

---

Ir 770 veces más rápido no sirve de nada si el cerebro simulado ya no es el del
artículo. Por eso existe [`validate_brian2.py`](../../../desktop/fly_brain/validate_brian2.py):
corre **el mismo experimento** con el `model.py` original de Shiu et al. (Brian2) y
con nuestro simulador, y compara **neurona a neurona**.

## Cómo funciona

1. Descarga el `model.py` original (licencia MIT) a `data/reference/`.
2. Prepara el compilador de C++ de Visual Studio que necesita Brian2
   (`msvc_env.py`).
3. Corre el experimento con el código original: estimula las neuronas elegidas a
   150 Hz durante 1 s, en varios ensayos.
4. Corre el mismo experimento con nuestro simulador (con **más** ensayos, porque son
   baratos y así hay menos ruido Poisson).
5. Compara la frecuencia media de **cada neurona** en los dos, **excluyendo las
   estimuladas** (triviales: disparan a 150 Hz en ambos).

```bash
python validate_brian2.py                  # looming (LPLC2 + LC4), 8 ensayos
python validate_brian2.py LC10a --trials 4 # cualquier otro tipo
python validate_brian2.py --reuse          # reutiliza el resultado de Brian2 guardado
```

## El resultado

*Looming* (LPLC2 + LC4, 150 Hz, 1 s):

| | Original (Brian2, 8 ensayos) | Nuestro (GPU, 30 ensayos) |
|---|---|---|
| Tiempo | 740 s | 3,6 s |
| Correlación neurona a neurona | — | **r = 0,999** |
| Pendiente (nuestro / original) | — | **1,003** |
| Giant Fiber (DNp01 derecha) | 195,6 Hz | 193 Hz |

- **r = 0,999**: el patrón de qué neuronas se activan y cuánto es prácticamente
  idéntico en todo el cerebro.
- **Pendiente 1,003**: no hay sesgo sistemático; no disparamos ni más ni menos.
- La Giant Fiber, la neurona que decide el escape, coincide en un ~1 %, dentro del
  ruido Poisson entre ensayos.

`validate_brian2.py` guarda las frecuencias de ambos en `data/validation_*.npz`, y
`python figures.py scatter` dibuja la comparación neurona a neurona (cada punto, una
neurona; la diagonal, coincidencia perfecta).

---

## Los dos detalles de Brian2

La primera versión de nuestro simulador ya tenía las ecuaciones bien, pero daba
**claramente más actividad** que el original. Hubo que encontrar dos diferencias
sutiles en cómo Brian2 interpreta el modelo:

### 1. La entrada durante el refractario se descarta

En el modelo original la conductancia está declarada así:

```
dg/dt = -g / tau : volt (unless refractory)
```

El `(unless refractory)` significa que, mientras la neurona es refractaria, `g` no
cambia **en absoluto**: ni decae ni acumula lo que llega. La entrada sináptica que
llega en ese intervalo **se pierde**.

Nuestra primera versión la acumulaba y la soltaba al salir del refractario. Eso
daba entre un 20 y un 40 % más de actividad según el experimento. Ahora:

```python
g[i] = gn + row[i] if act else gi   # en refractario la entrada se descarta
row[i] = 0.0                        # y se borra igualmente
```

### 2. El refractario dura 21 pasos, no 22

`t_rfc / dt = 2,2 / 0,1 = 22`. Lo natural es congelar 22 pasos. Pero Brian2 vuelve a
integrar la neurona cuando **han pasado** `round(t_rfc/dt)` pasos desde la espiga,
contando el propio paso de la espiga. En la práctica congela **uno menos**:

```python
def refractory_steps(p):
    return max(0, int(round(p.t_rfc / p.dt)) - 1)   # 21 con dt = 0,1 ms
```

Un paso de diferencia (0,1 ms) parece nada, pero en neuronas que disparan a
200 Hz (cada 5 ms) es un 2 % del ciclo, y se nota en la correlación.

### Y el orden de las operaciones

Integrar → umbral → llegan las sinapsis → reset, igual que Brian2. Lo que llega en
el mismo paso en que la neurona dispara se borra con el reset. Ver
[el modelo LIF](02-modelo-lif.md#el-orden-dentro-de-un-paso).

---

## Tests automáticos

Además de la validación contra el original (que necesita Brian2 y un compilador),
[`tests/test_lif.py`](../../../desktop/fly_brain/tests/test_lif.py) comprueba el
simulador sobre **redes pequeñas hechas a mano**, sin descargar FlyWire. Las mismas
pruebas se corren contra `Brain` (CPU) y, si hay GPU, contra `BrainGPU`:

| Test | Qué comprueba |
|---|---|
| `test_rest_is_silent` | Sin estímulo, nadie dispara |
| `test_poisson_stimulus_rate` | Estimular a 100 Hz da ~100 Hz (±10) |
| `test_strong_synapse_relays_after_delay` | 200 sinapsis hacen disparar a la diana, y no antes de `t_dly` |
| `test_weak_synapse_stays_subthreshold` | 20 sinapsis despolarizan pero no disparan |
| `test_inhibition_blocks_relay` | Una inhibición fuerte bloquea el relevo |
| `test_refractory_caps_rate` | Nadie supera 1 / `t_rfc` |
| `test_coarser_dt_keeps_relay` | Con `dt` 0,1 y 0,5 la cadena se mantiene |

```bash
cd desktop/fly_brain
.venv\Scripts\python -m unittest discover -s tests -v
```

---

[← Cómo reducimos el tiempo](05-reduccion-del-tiempo.md) · [Índice](README.md) · [Siguiente: el piloto →](07-piloto.md)
