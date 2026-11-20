# 9 · Tools, tests and glossary

> 🌐 **English** · [Español](../cerebro/09-herramientas-y-glosario.md)

[← Watching the brain live](08-visualization.md) · [Index](README.md)

---

## `probe.py`: experimenting with the brain

[`probe.py`](../../../desktop/fly_brain/probe.py) stimulates a cell type from rest and
shows which **motor descending neurons** respond. It's the tool used to decide which
neurons the pilot uses.

```bash
python probe.py LPLC2 LC4              # looming in both eyes -> Giant Fiber
python probe.py LC10a --side left      # object on the left -> left DNa02
python probe.py LC9 --hz 100 --ms 300  # custom rate and duration
python probe.py --device cpu --dt 0.5  # force CPU and a coarse step
python probe.py --scan                 # every sensory type -> data/scan.csv
```

Typical output (the script prints in Spanish):

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

(Exact numbers depend on the stimulus and the seed.)

The descending neurons it watches, with their known function in the literature
(Namiki 2018, Bidaye 2014/2020, Cande 2018…):

| Type | Function |
|---|---|
| DNp01 | Giant Fiber: escape (jump) |
| DNp09 | P9: walk forward |
| DNa02 | Turn towards its side |
| DNa01 | Turning / steering |
| MDN | *Moonwalker*: walk backwards |
| DNp02, DNp04, DNp11 | Looming response |
| DNg11 | Front-leg grooming |

Without a GPU it uses `dt = 0.5 ms` to run in real time (< 5 % difference in these
tests).

---

## The 673-type sweep

`probe.py --scan` stimulates, **one at a time and per side**, every sensory and
visual-projection type with at least 2 neurons (673 stimuli, 150 Hz, 300 ms each) and
saves the response of all the motor descending neurons to `data/scan.csv`.

That's 673 × 300 ms ≈ **200 s of fly life**, simulated on the GPU in **~45 s** (thanks
to `reset()` reusing the same brain and the same CUDA Graph).

What came out:

| Stimulus | Descending | Behaviour |
|---|---|---|
| LPLC2, LC4, LC6 | DNp01 (Giant Fiber), 110–170 Hz | Escape from something approaching |
| LPLC2 + LC4 on one side | DNp02/04/11 on the same side | Lateralized response |
| LC10a / c-2 / d on one side | DNa02 on the same side, 77–133 Hz | Pursue an object |
| LC9, LC31a | DNp09 (P9), 50–92 Hz | Walk forward |
| LPC1, LC9, LC6 | MDN, 15–32 Hz | Walk backwards |

The [pilot](07-pilot.md)'s choice of inputs and outputs comes straight from this
table.

---

## `benchmark.py`: measuring performance

[`benchmark.py`](../../../desktop/fly_brain/benchmark.py) measures the figures in
[how we cut the time](05-cutting-the-time.md):

| Measure | What it gives |
|---|---|
| `speed` | Times real time and µs per step, on CPU and GPU, at `dt` 0.1 and 0.5 ms |
| `activity` | Spikes per step and synapses delivered per step: the **real** saving of event-driven propagation |
| `graphs` | GPU with and without CUDA Graphs |
| `branches` | CPU: branch-free integration versus the `if` version |

```bash
python benchmark.py                   # everything
python benchmark.py --only activity   # one measure
python benchmark.py --markdown        # tables to paste into the wiki
python benchmark.py --synthetic       # random network, no data (only to test the script)
```

Results are saved to `data/benchmark.json`.

## `figures.py`: the wiki's charts

[`figures.py`](../../../desktop/fly_brain/figures.py) generates the images in
`docs/wiki/assets/`, in Spanish and English:

| Figure | Command | Needs |
|---|---|---|
| Speed of each version | `python figures.py speed` | Nothing |
| Original vs ours, neuron by neuron | `python figures.py scatter` | Having run `validate_brian2.py` |
| Spikes during an escape (*raster*) | `python figures.py raster` | The FlyWire connectome |

## All the tests

```bash
cd desktop/fly_brain
.venv\Scripts\python -m unittest discover -s tests -v     # 31: simulator (CPU and GPU), pilot and world
cd ..
fly_brain\.venv\Scripts\python -m unittest discover -s tests -v   # 17: kinematics and gait
```

- **Simulator:** LIF on CPU and GPU, with small hand-built networks (see
  [validation](06-validation.md#automated-tests)).
- **Pilot:** the integration step per hardware (0.1 ms on GPU, 0.5 ms on CPU) and a
  bounded command.
- **World:** cave geometry, LiDAR, duplicate-free map, gas, heat and wall avoidance;
  in the real cave, the route, the LiDAR and the gas.

---

## FAQ

**Does the fly "learn" to explore the cave?**
No. There is no training and no plasticity: the weights are the measured connectome
and never change. The behaviour comes from how the real brain is wired; the pilot
only chooses where information goes in and what is read out.

**Is it "a fly's brain" for real?**
It is the complete, real wiring of a fly brain, with a very simplified neuron model
(LIF: no dendrites, no neuromodulators, no adaptation). That model reproduces
behaviours measured in living flies (Shiu et al. 2024), but it is not a full
biophysical simulation.

**Does it drive the physical robot?**
Not yet. The fly brain pilots the simulated Spot Micro in the cave viewer only. The
physical robot is driven from the GUI and carries no mission sensors, which were left
out on purpose so it can be adapted to different projects. See
[from simulation to the real robot](10-from-simulation-to-the-real-robot.md).

**Do I need a GPU?**
It's not required: everything runs on the CPU. Without a GPU the pilot uses
`dt = 0.5 ms`, at which the CPU runs at ~1× (the viewer, with drawing on top, somewhat
below). With an NVIDIA GPU and `cupy-cuda12x[ctk]` it uses the paper's `dt` (0.1 ms)
and runs 8 times ahead of real time.

**Why not use Brian2, NEST or GeNN directly?**
Brian2 is the original and needs to compile C++ for each experiment; it's also built
for complete runs, not for advancing 40 ms and handing back control. A ~400-line
custom simulator, validated against the original, plugs straight into the robot loop
and adds no heavy dependencies.

**Why does gas go in through the eyes?**
Because the model's olfactory and thermal pathways flood the brain and give no
direction. See [the pilot](07-pilot.md#why-go-in-through-the-eyes).

---

## Glossary

| Term | Meaning |
|---|---|
| **Connectome** | Complete map of the connections between the neurons of a nervous system |
| **FlyWire** | Consortium that reconstructed the full adult *Drosophila* brain connectome (v783) |
| **LIF** | *Leaky Integrate-and-Fire*: a neuron modelled as a leaky capacitor that fires when crossing a threshold |
| **Spike** | Action potential: a neuron's "firing" |
| **Refractory period** | Time after a spike during which the neuron cannot fire again |
| **Synaptic delay** | Time for a spike to reach its target (1.8 ms here) |
| **Poisson** | Random process of independent events at an average rate; here, the stimulation |
| **Optogenetics** | Technique that activates specific neurons with light; the model imitates it with Poisson input |
| **Descending neuron (DN)** | Neuron that runs from the brain to the nerve cord and controls the body |
| **Giant Fiber (DNp01)** | The descending neuron that triggers the fly's escape jump |
| **P9 (DNp09)** | Descending neuron associated with walking forward |
| **MDN** | *Moonwalker descending neuron*: walking backwards |
| **LC, LPLC** | *Lobula columnar*, *lobula plate–lobula columnar*: visual projections from the optic lobe to the central brain |
| **Looming** | Visual stimulus of something growing fast on the retina: it's approaching |
| **Sensory substitution** | Presenting information through a sense other than the original one (here, gas and heat as visual scenes) |
| **CSC** | *Compressed Sparse Column*: sparse matrix format with each column contiguous in memory |
| **Event-driven propagation** | Delivering only the output of neurons that fired, instead of multiplying the whole matrix |
| **Kernel** | Function that runs on the GPU, in thousands of threads at once |
| **Warp** | Group of 32 GPU threads that execute the same instruction together |
| **`atomicAdd`** | Memory addition that isn't lost even when many threads write to the same address at once |
| **CUDA Graph** | Sequence of kernel launches recorded once and replayed with a single call |
| **SIMD** | *Single Instruction, Multiple Data*: a CPU instruction that operates on several numbers at once |
| **Subnormal** | Extremely small floating-point number, very slow to operate on in many CPUs |
| **numba** | JIT compiler that turns Python functions into machine code |
| **CuPy** | NumPy-like library for NVIDIA GPUs, which also compiles CUDA kernels written in C |
| **Brian2** | Python neural network simulator, used by the original model |

---

## References

- Dorkenwald et al. 2024. *Neuronal wiring diagram of an adult brain.* **Nature** 634.
- Schlegel et al. 2024. *Whole-brain annotation and multi-connectome cell typing of
  Drosophila.* **Nature** 634.
- Shiu et al. 2024. *A Drosophila computational brain model reveals sensorimotor
  processing.* **Nature**. Code: [philshiu/Drosophila_brain_model](https://github.com/philshiu/Drosophila_brain_model) (MIT).
- Annotations: [flyconnectome/flywire_annotations](https://github.com/flyconnectome/flywire_annotations) (CC BY 4.0).
- Skeletons: FlyWire 783, Jefferis group (MRC LMB), CC BY 4.0.

---

[← Watching the brain live](08-visualization.md) · [Index](README.md) · [Next: from simulation to the real robot →](10-from-simulation-to-the-real-robot.md)
