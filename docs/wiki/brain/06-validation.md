# 6 · Validation against the original model

> 🌐 **English** · [Español](../cerebro/06-validacion.md)

[← How we cut the time](05-cutting-the-time.md) · [Index](README.md) · [Next: the pilot →](07-pilot.md)

---

Going 770 times faster is worthless if the simulated brain is no longer the one from
the paper. That is why [`validate_brian2.py`](../../../desktop/fly_brain/validate_brian2.py)
exists: it runs **the same experiment** with Shiu et al.'s original `model.py`
(Brian2) and with our simulator, and compares them **neuron by neuron**.

## How it works

1. Downloads the original `model.py` (MIT licence) to `data/reference/`.
2. Sets up the Visual Studio C++ compiler Brian2 needs (`msvc_env.py`).
3. Runs the experiment with the original code: stimulates the chosen neurons at
   150 Hz for 1 s, over several trials.
4. Runs the same experiment with our simulator (with **more** trials, since they are
   cheap and that means less Poisson noise).
5. Compares the mean firing rate of **every neuron** in both, **excluding the
   stimulated ones** (trivial: they fire at 150 Hz in both).

```bash
python validate_brian2.py                  # looming (LPLC2 + LC4), 8 trials
python validate_brian2.py LC10a --trials 4 # any other type
python validate_brian2.py --reuse          # reuse the saved Brian2 result
```

## The result

*Looming* (LPLC2 + LC4, 150 Hz, 1 s):

| | Original (Brian2, 8 trials) | Ours (GPU, 30 trials) |
|---|---|---|
| Time | 740 s | 3.6 s |
| Neuron-by-neuron correlation | — | **r = 0.999** |
| Slope (ours / original) | — | **1.003** |
| Giant Fiber (DNp01 right) | 195.6 Hz | 193 Hz |

- **r = 0.999**: the pattern of which neurons activate and by how much is practically
  identical across the whole brain.
- **Slope 1.003**: no systematic bias; we fire neither more nor less.
- The Giant Fiber, the neuron that decides the escape, matches within ~1 %, inside
  the trial-to-trial Poisson noise.

`validate_brian2.py` saves both sets of rates to `data/validation_*.npz`, and
`python figures.py scatter` plots the neuron-by-neuron comparison (each dot, one
neuron; the diagonal, a perfect match).

---

## The two Brian2 details

The first version of our simulator already had the right equations, but produced
**clearly more activity** than the original. Two subtle differences in how Brian2
interprets the model had to be found:

### 1. Input during the refractory period is discarded

In the original model the conductance is declared like this:

```
dg/dt = -g / tau : volt (unless refractory)
```

`(unless refractory)` means that while the neuron is refractory, `g` does not change
**at all**: it neither decays nor accumulates what arrives. Synaptic input arriving
in that window **is lost**.

Our first version accumulated it and released it when the refractory period ended.
That gave between 20 and 40 % more activity depending on the experiment. Now:

```python
g[i] = gn + row[i] if act else gi   # input during refractory is discarded
row[i] = 0.0                        # and cleared anyway
```

### 2. The refractory period lasts 21 steps, not 22

`t_rfc / dt = 2.2 / 0.1 = 22`. The natural thing is to freeze 22 steps. But Brian2
resumes integrating when `round(t_rfc/dt)` steps **have passed** since the spike,
counting the spike's own step. In practice it freezes **one fewer**:

```python
def refractory_steps(p):
    return max(0, int(round(p.t_rfc / p.dt)) - 1)   # 21 at dt = 0.1 ms
```

One step (0.1 ms) seems like nothing, but for neurons firing at 200 Hz (every 5 ms)
it is 2 % of the cycle, and it shows in the correlation.

### And the order of operations

Integrate → threshold → synaptic input arrives → reset, exactly like Brian2. Input
arriving in the same step a neuron fires is wiped by the reset. See
[the LIF model](02-lif-model.md#order-of-operations-within-a-step).

---

## Automated tests

Besides validation against the original (which needs Brian2 and a compiler),
[`tests/test_lif.py`](../../../desktop/fly_brain/tests/test_lif.py) checks the
simulator on **small hand-built networks**, without downloading FlyWire. The same
tests run against `Brain` (CPU) and, if there is a GPU, against `BrainGPU`:

| Test | What it checks |
|---|---|
| `test_rest_is_silent` | With no stimulus, nobody fires |
| `test_poisson_stimulus_rate` | Stimulating at 100 Hz gives ~100 Hz (±10) |
| `test_strong_synapse_relays_after_delay` | 200 synapses make the target fire, and not before `t_dly` |
| `test_weak_synapse_stays_subthreshold` | 20 synapses depolarize but don't fire |
| `test_inhibition_blocks_relay` | Strong inhibition blocks the relay |
| `test_refractory_caps_rate` | Nobody exceeds 1 / `t_rfc` |
| `test_coarser_dt_keeps_relay` | At `dt` 0.1 and 0.5 the chain holds |

```bash
cd desktop/fly_brain
.venv\Scripts\python -m unittest discover -s tests -v
```

---

[← How we cut the time](05-cutting-the-time.md) · [Index](README.md) · [Next: the pilot →](07-pilot.md)
