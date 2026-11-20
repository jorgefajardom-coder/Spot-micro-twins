# 3 · The CPU simulator (`lif.py`)

> 🌐 **English** · [Español](../cerebro/03-simulador-cpu.md)

[← The LIF model](02-lif-model.md) · [Index](README.md) · [Next: the GPU simulator →](04-gpu-simulator.md)

---

## Why rewrite it

Shiu et al.'s original model is written with **Brian2**, an excellent research
simulator, but it had three problems for this project:

1. **It needs a C++ compiler.** Brian2 generates C++ code and compiles it. On
   Windows that means installing Visual Studio and fighting its detection (which is
   what `msvc_env.py` does; it is now only used for validation).
2. **It is designed to run a whole experiment** (`run(1 s)`) and return the spikes
   at the end, not to **advance 40 ms, hand control back to the robot, change the
   stimulus and carry on**, 24 times per second.
3. **It is slow for this use:** see [the comparison](05-cutting-the-time.md#the-starting-point).

So [`lif.py`](../../../desktop/fly_brain/lif.py) reimplements the same model in
Python + **numba** (compiles Python to machine code with LLVM, no external C++
compiler).

---

## The API

```python
from connectome import Connectome
from lif import Params, make_brain

c = Connectome.load()
brain = make_brain(c, Params(dt=0.1))      # GPU if CUDA is available, otherwise CPU
brain.stimulate((c.find('LPLC2'), 150),    # (indices, Hz) pairs
                (c.find('LC4'), 150))
counts = brain.run(100)                    # spikes per neuron in 100 ms
brain.rate(counts, 100, c.find('DNp01'))   # mean Hz of the Giant Fiber
brain.reset()                              # back to rest, same object
```

- `stimulate()` **replaces** the previous stimulation (no arguments turns it off).
- `run(ms)` advances and returns **how many spikes** each neuron fired in that span.
- The state (`v`, `g`, refractory counters, delay buffer) **persists** between calls:
  the brain stays "alive" from one frame to the next.

`make_brain()` chooses by itself: if `lif_gpu.available()` it returns a `BrainGPU`;
otherwise a CPU `Brain`. Both have the same API and pass the same tests.

---

## Data layout

Everything is flat NumPy arrays, one per variable (*structure of arrays*):

| Array | Type | Why that type |
|---|---|---|
| `v`, `g` | `float32` | Plenty of precision for mV; half the memory of `float64` |
| `ref_left` | `int16` | Remaining refractory steps (max 21) |
| `no_refractory` | `bool` | Stimulated neurons have no refractory period |
| `pending` | `float32[18, n]` | Ring buffer for the synaptic delay |
| `w_data` | `float32` | Weights **already multiplied by `w_syn`** (one fewer multiplication per synapse) |
| `w_indices`, `w_indptr` | `int32`, `int64` | The CSC matrix |
| `spike_count` | `int64` | Cumulative spike counter |

---

## A step, in two phases

### Phase 1 · `_integrate`: every neuron, no branches

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

Notice what is **not** there: no per-neuron `if ... continue`. Everything is computed
for every neuron and then **selected** with conditional expressions, which the
compiler turns into *select* / *blend* instructions. That lets LLVM **vectorize** the
loop with SIMD (AVX2: 8 neurons per instruction in `float32`).

> On the development PC, the version with per-neuron shortcuts was **~7 times
> slower** (as noted in the code): which neurons are refractory is random, the branch
> predictor keeps missing, and the loop stops vectorizing. Note: the effect depends on
> the CPU and the compiler; sometimes LLVM turns the branch into a select by itself and
> both versions end up equal. `python benchmark.py --only branches` measures it on
> your machine.
> See [optimization 4](05-cutting-the-time.md#4-branch-free-loop-simd-on-the-cpu).

### Phase 2 · `_run`: stimuli, reset and propagation

```python
for j in range(len(stim_idx)):                 # stimulated neurons only
    if random() < stim_p[j]:
        v[stim_idx[j]] += w_poi
for pre in range(n):
    if fired[pre]:                             # almost always False
        v[pre] = v_rst; g[pre] = 0
        if not no_refractory[pre]:
            ref_left[pre] = ref_steps
        counts[pre] += 1
        for k in range(indptr[pre], indptr[pre+1]):   # only its column
            row[indices[k]] += data[k]
```

There is a branch here, but it is **very predictable**: only a tiny fraction of
neurons fire in any step, so the predictor guesses "doesn't fire" almost every
time. And when one does fire, only **its column** of the connectome is walked: that
is [event-driven propagation](05-cutting-the-time.md#1-event-driven-propagation),
the most important optimization of all.

---

## Other details that add up

- **`@njit(cache=True)`**: the compiled code is saved to disk; the second run does
  not recompile (saves seconds at startup).
- **`fastmath=True`** in `_integrate`: lets LLVM reorder floating-point operations
  to vectorize better.
- **`G_EPS = 1e-4` mV**: when `g` decays below this it is set to 0. Otherwise, after
  many steps of exponential decay, `g` reaches **subnormal** numbers
  (< 1.2·10⁻³⁸), which on x86 are tens of times slower to operate on. 10⁻⁴ mV is
  negligible next to the 7 mV to threshold.
- **A single Python call per run:** `run(ms)` enters numba once and does all the
  steps inside; Python never sees individual steps.
- **Reproducible seed:** `_seed()` seeds numba's generator (separate from NumPy's) so
  two runs with the same seed give the same spikes.

---

## Performance

On an i7-12650H, with *looming* (LPLC2 + LC4 at 150 Hz):

| `dt` | Speed | Cost per step |
|---|---|---|
| 0.1 ms (original) | **0.29×** real time | ~345 µs |
| 0.5 ms | **0.99×** real time | — (5× fewer steps) |

With `dt = 0.5 ms` the CPU is practically real time and firing rates change by only
**3 %**. That is why `probe.py` and the viewer's pilot (`FlyPilot`) use 0.5 ms
automatically when there is no GPU. Running 8 times
*faster* than real time with the paper's `dt` needs the GPU →
[next page](04-gpu-simulator.md).

---

[← The LIF model](02-lif-model.md) · [Index](README.md) · [Next: the GPU simulator →](04-gpu-simulator.md)
