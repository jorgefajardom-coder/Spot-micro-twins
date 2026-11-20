# 12 · Troubleshooting

> 🌐 **English** · [Español](../cerebro/12-solucion-de-problemas.md)

[← Adding a new behaviour](11-adding-a-behaviour.md) · [Index](README.md) · [Next: limitations and future work →](13-limitations-and-future-work.md)

---

## The brain isn't using the GPU

At startup the viewer says where it runs:

```
Brain on GPU · dt 0.1 ms · calibration {...}     ← good
Brain on CPU · dt 0.5 ms · calibration {...}     ← no GPU found
```

Check in the `fly_brain` environment:

```bash
.venv\Scripts\python -c "import lif_gpu; print(lif_gpu.available())"
```

| If you get… | Likely cause | Fix |
|---|---|---|
| `ModuleNotFoundError: cupy` | CuPy not installed | `pip install "cupy-cuda12x[ctk]"` in fly_brain's `.venv` |
| `False` with no error | No visible NVIDIA GPU, or an old driver | Update the NVIDIA driver (CUDA 12 needs a recent one); check with `nvidia-smi` |
| Error compiling the kernel | The CUDA compiler is missing | Install with `[ctk]` (brings NVRTC from pip) rather than plain `cupy-cuda12x` |
| `CUDA path could not be detected` | CuPy warning | Harmless with `[ctk]`; silenced automatically |

To force one or the other: `--device gpu` (fails if there is none) or `--device cpu`.

## The viewer is slow

1. **See what's taking the time:**
   ```bash
   fly_brain\.venv\Scripts\python spot_brain.py --profile
   ```
   When the window closes it prints the median and 95th percentile of each phase
   (world + LiDAR, brain, gait, draw, render).
2. **If "brain" is the most expensive:** the brain is on the CPU. See the previous
   section. Without a GPU, the pilot already uses `dt = 0.5 ms` to get close to real
   time.
3. **If "render (VTK)" is the most expensive:** it's almost always the **integrated
   GPU**. On Windows: *Settings → Display → Graphics*, add the `.venv`'s `python.exe`
   and choose **High performance** (the NVIDIA one). On the integrated GPU the viewer
   runs at under half the frame rate.
4. **If it's "draw":** close the tabs you aren't looking at (only visible ones are
   drawn) or lower `ACTIVE_SEGMENTS` / `MAX_SYNAPSE_SOURCES` in `spot_brain.py`.

## It takes a long time to start

- **The first time**, numba compiles the functions (tens of seconds) and the connectome
  cache is built. Later runs are much faster.
- If it's **always** slow, numba's cache may not be writable (read-only folder) or may
  be corrupted: delete the `__pycache__` folders in `fly_brain/` and run again.
- If you changed the FlyWire data, delete `data/connectome_cache.npz` so it's rebuilt.

## Out of GPU memory

The brain needs ~140 MB of GPU memory (weights + state). If it fails with
`OutOfMemoryError`:

- Close other programs using the GPU (games, other notebooks with CuPy/PyTorch).
- Don't create several `Brain`s at once: use `reset()` to repeat experiments with the
  same object.

## It can't find the data

```
FileNotFoundError: .../fly_brain/data/neuron_annotations.tsv
```

The data isn't in the repository (~130 MB). Download it:

```bash
.venv\Scripts\python download_data.py
```

Without `skeletons_lite.npz` the viewer works the same, but draws the brain as a point
cloud. For the real shapes: `python download_skeletons.py` (~1 h, can be interrupted
and resumed).

## `probe.py` can't find a cell type

```
No hay neuronas de tipo ['LC10'] (ambos lados)
```

Names are exact (`LC10a`, not `LC10`). To search by prefix:

```python
c.find_prefix('LC10')
c.annotations[c.annotations.cell_type.str.startswith('LC10', na=False)].cell_type.unique()
```

## `validate_brian2.py` fails on Windows

Brian2 needs Visual Studio's C++ compiler. Install *Visual Studio Build Tools* with the
**"Desktop development with C++"** workload. `msvc_env.py` finds it with `vswhere` and
loads its environment automatically.

## The robot always turns to one side

The connectome isn't symmetric and calibration compensates at startup. If it's off
(e.g. after changing `INPUTS`), check that `calibrate()` is called before the loop and
look at the gains it prints: a side stuck at the minimum gain (10) usually means that
group has no neurons on that side.

---

[← Adding a new behaviour](11-adding-a-behaviour.md) · [Index](README.md) · [Next: limitations and future work →](13-limitations-and-future-work.md)
