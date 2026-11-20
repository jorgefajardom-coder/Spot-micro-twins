# 8 · Watching the brain live

> 🌐 **English** · [Español](../cerebro/08-visualizacion.md)

[← The pilot](07-pilot.md) · [Index](README.md) · [Next: tools and glossary →](09-tools-and-glossary.md)

---

Tab **1 · Brain · LiDAR map** in [`spot_brain.py`](../../../desktop/spot_brain.py)
shows the fly brain in 3D while it pilots, with firing neurons lit up and the
synapses the signal travels along. Drawing 138,639 neurons at ~30 fps alongside the
robot, the cave and the map needed its own tricks too.

---

## The shape of each neuron

### With morphology (recommended)

[`download_skeletons.py`](../../../desktop/fly_brain/download_skeletons.py) downloads
the FlyWire 783 **skeletons** (published by the Jefferis group, MRC LMB): for each
neuron, a graph of points and edges tracing its axon and dendrites.

Raw, they are unmanageable (~1,700 vertices per neuron × 138,639 ≈ **235 million**
vertices), so they are simplified in two passes by clustering vertices on a grid:

| Pass | Cell | When |
|---|---|---|
| 1 | 3 µm | At download, in chunks of 2,000 neurons (`data/skeletons/part_*.npz`) |
| 2 | 10 µm | When merging (`--merge-only`), vectorized over all neurons at once |

Result: `skeletons_lite.npz` with **~8.1 million segments**, which the viewer animates
in real time.

- Downloads with **48 threads** in parallel, with retries.
- **Resumable:** each chunk is saved when done; if interrupted, it continues from the
  next one (~1 h and ~6 GB of traffic in total).
- `python download_skeletons.py --limit 5000` fetches a sample for testing.

### Without morphology

If the skeletons haven't been downloaded, the viewer draws a **point cloud**, one per
neuron, at its soma position. That already shows the brain's shape.

### FlyWire-style colours

One colour per **cell type** (from the `crc32` of the name, so it is stable between
runs), with a palette per region: cyan, turquoise and yellow in the **optic lobes**;
violet, magenta and blue in the **central brain**. Midnight-blue background, fixed
front view filling the panel (drag with the mouse to rotate).

---

## Activity

Every frame, the pilot returns every neuron's spikes (`pilot.last_counts`). The
viewer keeps a **visible activity** per neuron that lights up with each spike and
fades exponentially:

```python
activity *= exp(-dt_ms / 120)               # fades in ~120 ms
activity  = min(1, activity + 0.5 * spikes)
```

Neurons with activity > 0.12 are drawn **in orange with their full morphology**, on an
**overlay layer** (`SetLayer(1)`) that shares the brain's camera: that way they glow
*inside* the brain even when other neurons are in front.

### Why there's a cap

A few wide-field neurons (spanning half the brain) would be enough to paint it all
orange and sink the frame rate. So:

- The **600 most active** are taken as candidates.
- They are drawn, most active first, up to a **budget of 15,000 segments**.

### The performance trick: the geometry doesn't move

The 8.1 M morphology points are uploaded to the GPU **once**. Each frame only changes
**which segments** make up the active layer (a list of indices) and their colour
value. Since segments are **sorted by neuron**, those of the active neurons are
computed with vectorized arithmetic, no loops:

```python
counts  = sk["count"][on]
seg_idx = np.repeat(sk["start"][on] - np.cumsum(counts) + counts, counts) + np.arange(counts.sum())
```

---

## Synapses

From each neuron that just fired, lines go to its **3 strongest targets** in the real
connectome: **orange** if the connection is excitatory, **blue** if inhibitory.

- The strongest targets of **all** neurons are precomputed **once** at startup
  (`top_targets()`, in numba): a 138,639 × 3 table. Each frame just indexes it.
- At most **220 neurons** draw their synapses per frame (the most active ones).
- Lines start from the **centre of each neuron's morphology** (inside the neuropil,
  where the real synapses are), not from the soma, which sits in the outer cortex.

---

## The other tabs

| Tab | Top | Bottom |
|---|---|---|
| **2 · Eyes · Descending** | Stimulus Hz for each visual group (`cmd.inputs`) | Hz for each descending group (`cmd.rates`) |
| **3 · History · Sensors** | Last 8 s of forward, turn, P9 and Giant Fiber | Gas (ppm) and temperature (°C) |

They are a direct window into the pilot: you see in real time what the robot "shows"
the fly and how it answers.

---

## Viewer performance

On an RTX 4060 Laptop with an i7-12650H: **~28 ms median per frame**, slowest 48 ms.

| Optimization | Why |
|---|---|
| **Meshes updated in place** (`LiveLines`, `LivePoints`, `copy_from`) | Recreating VTK actors every frame was the biggest cost: **11.6 → 41 fps** |
| **LiDAR map in another process** (`ProcessPoolExecutor`) | Rebuilding the surface froze the frame for ~70 ms every 1.5 s |
| **Only visible tabs are drawn** | Hidden charts keep accumulating data but aren't rendered |
| **Brain geometry fixed on the GPU** | Only indices and colours change |
| **Activity and synapse caps** | Bounded cost even if half the brain lights up |
| **FXAA** instead of supersampling | Supersampling doubled VTK's text and broke SSAO |
| **Real `dt`, clamped to 20–80 ms** | If a frame is slow, the brain advances that much time (no lag build-up) but without huge jumps |

To see where each frame's time goes: `spot_brain.py --profile` prints, on close, the
median and 95th percentile of each phase (world + LiDAR, brain, gait, draw, render).

To record video (`--record`), `dt` is fixed (1/24 s) and `--speed` sets how many
simulation steps go into each frame.

---

[← The pilot](07-pilot.md) · [Index](README.md) · [Next: tools and glossary →](09-tools-and-glossary.md)
