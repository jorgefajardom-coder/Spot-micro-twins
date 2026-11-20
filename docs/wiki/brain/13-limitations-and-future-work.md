# 13 · Limitations and future work

> 🌐 **English** · [Español](../cerebro/13-limitaciones-y-trabajo-futuro.md)

[← Troubleshooting](12-troubleshooting.md) · [Index](README.md)

---

## Known limitations

### Of the neuron model

| Limitation | Consequence |
|---|---|
| **Point LIF**: no dendrites, no detailed ion channels | Reproduces measured rates and behaviours, but not fine biophysics |
| **No adaptation** | A stimulated neuron keeps firing the same forever; highly recurrent circuits run away |
| **No neuromodulators** (dopamine, octopamine…) | No internal states: hunger, sleep, arousal |
| **No plasticity** | The fly doesn't learn; weights are always the connectome's |
| **Sign from the predicted neurotransmitter** | Some neurons may have the wrong sign (glutamate, for instance, can be excitatory at some receptors) |
| **Brain only** | The ventral nerve cord isn't included; that's why descending neurons are read directly and `gait.py` acts as the spinal cord |

### Of the robot integration

- **Smell and heat don't work as-is:** they flood the brain and give no direction (see
  [the pilot](07-pilot.md#why-go-in-through-the-eyes)). Sensory substitution through
  visual circuits is used instead.
- **The physical robot carries no mission sensors:** gas, ambient temperature and LiDAR
  **were left out on purpose**, so the platform can be adapted to different projects.
  In the simulation they are virtual sensors. See
  [from simulation to the real robot](10-from-simulation-to-the-real-robot.md).
- **The brain only pilots the simulation:** the physical robot is driven from the GUI.
- **The arm doesn't manipulate:** it only orients the head and the LiDAR, and doesn't
  check collisions with the ceiling.
- **The world knows where the robot is:** the simulation doesn't need to estimate its
  position; a real robot does.

### Of performance

- **Without an NVIDIA GPU**, the brain runs on the CPU at `dt = 0.5 ms` (~1× for the
  simulation alone). With drawing on top, the viewer stays somewhat below real time.
- **CUDA only:** there is no version for AMD or Apple GPUs (CuPy has experimental ROCm
  support, untested here).
- **Some figures in this wiki haven't been measured on the development PC yet** with
  `benchmark.py` (spikes per step, the exact effect of CUDA Graphs and of removing
  branches). See [measure it yourself](05-cutting-the-time.md#measure-it-yourself).

---

## Future work

### Brain

- **Improve the antennal lobe** (lateral inhibition, adaptation) to use smell for real
  instead of sensory substitution.
- **Per-neuron adaptation** (a slow current that rises with each spike): the model
  would be more stable and allow longer continuous stimuli.
- **Connect the ventral nerve cord** (MANC / FANC connectomes) so the legs are driven by
  real neurons too, instead of `gait.py`.

### Performance

- **`float16` weights** or 24-bit indices: less traffic in propagation.
- **Reorder neurons** (by region or with Cuthill–McKee) so each column's targets sit
  closer in memory.
- **Several trials at once** on the same GPU to speed up `probe.py --scan` and
  validation.

### Robot

- **Integrate each project's sensors** on the physical robot and write the sense
  adapter ([plan](10-from-simulation-to-the-real-robot.md#how-it-would-be-connected)).
- **Localization** with odometry + IMU, or SLAM with the LiDAR.
- **A real arm** with a gripper that manipulates objects.

---

[← Troubleshooting](12-troubleshooting.md) · [Index](README.md)
