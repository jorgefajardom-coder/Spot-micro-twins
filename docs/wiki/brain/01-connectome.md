# 1 · The connectome

> 🌐 **English** · [Español](../cerebro/01-conectoma.md)

[← Index](README.md) · [Next: the LIF model →](02-lif-model.md)

---

## What a connectome is

A connectome is the **complete map of which neuron connects to which** in a nervous
system. The adult fly's was completed in 2024 by the **FlyWire** consortium
(Dorkenwald et al., *Nature* 634): they sliced a brain into thousands of sections,
imaged them with an electron microscope and reconstructed every neuron and every
synapse, helped by neural networks and thousands of human proofreading edits.

The result (version **783**) is what we simulate:

- **138,639 neurons**, each with a unique identifier (`root_id`).
- **~15 million connected pairs** (pre → post), and for each pair how many synapses
  there are.
- For each neuron, its predicted **neurotransmitter**, and with it the **sign**:
  acetylcholine = excitatory (+), GABA and glutamate = inhibitory (−).
- **Annotations** (Schlegel et al. 2024): cell type (`LPLC2`, `DNp01`…), class
  (`visual_projection`, `descending`…), hemisphere (`left`/`right`) and soma
  position.

> There is no "learned" or trained parameter. The weights are literally
> *synapse count × sign*, measured in the tissue.

---

## The files

`download_data.py` fetches three public files into `desktop/fly_brain/data/`
(outside the repository, ~130 MB):

| File | Source | Contents |
|---|---|---|
| `Connectivity_783.parquet` | Shiu et al. 2024 (MIT) | One row per pair: `Presynaptic_Index`, `Postsynaptic_Index`, `Excitatory x Connectivity` (signed synapses) |
| `Completeness_783.csv` | Shiu et al. 2024 (MIT) | The ordered list of `root_id`s: row *i* is neuron *i* in the simulator |
| `neuron_annotations.tsv` | Schlegel et al. 2024 (CC BY 4.0) | Type, class, side, neurotransmitter and position of each neuron |

---

## From a table to a sparse matrix

`Connectome.load()` (in [`connectome.py`](../../../desktop/fly_brain/connectome.py))
turns the table into a **weight matrix W** of 138,639 × 138,639:

- **Rows = postsynaptic neuron**, **columns = presynaptic**.
- `W[post, pre]` = signed synapses from `pre` to `post`.
- Stored in **CSC** format (*Compressed Sparse Column*).

### Why sparse?

A dense 138,639² matrix in `float32` would take **~77 GB**. But of those 19.2 billion
cells only ~15 million hold anything: **0.08 %** occupancy. A sparse format stores
only the non-zeros:

| Array | Type | Approximate size |
|---|---|---|
| `data` (weight) | `float32` | 15 M × 4 B ≈ 60 MB |
| `indices` (row = post) | `int32` | 15 M × 4 B ≈ 60 MB |
| `indptr` (where each column starts) | `int64` | 138,640 × 8 B ≈ 1.1 MB |

About **120 MB** in total: it fits easily in any modern GPU's memory.

### Why CSC and not CSR?

Because the simulator always asks the same question: *"neuron `pre` just fired,
who does it reach?"*. In CSC, **all of a neuron's targets are contiguous in
memory**:

```
for k in range(indptr[pre], indptr[pre + 1]):
    input[indices[k]] += data[k]
```

Reading a column means walking a contiguous slice of `indices` and `data`, the
fastest thing a CPU or GPU can do. It is the basis of
[event-driven propagation](05-cutting-the-time.md#1-event-driven-propagation).

### `sum_duplicates()` and the cache

- If a pre→post pair appears several times, `sum_duplicates()` merges it into one
  weight.
- The first time, the result is saved to `data/connectome_cache.npz`. Later loads
  read it directly (three NumPy arrays) without re-reading the parquet or rebuilding
  the matrix.

---

## Finding neurons by name

```python
from connectome import Connectome
c = Connectome.load()

c.find('DNp01')                # Giant Fiber, both sides -> simulator indices
c.find('LC10a', side='left')   # left hemisphere only
c.find_prefix('LC10')          # LC10a, LC10b, LC10c-1, ...
```

`find()` looks in `cell_type` and also in `hemibrain_type` (the name the neuron had
in the earlier partial connectome, *hemibrain*), because the literature uses both.

> **Side convention.** The left hemisphere processes the left eye, so
> `side='left'` in FlyWire = **the robot's left side**.

---

## The neurons that matter for the robot

Of the 138,639, the pilot touches a few families directly:

| Family | Type | What they do in the real fly |
|---|---|---|
| **Input** | `LC10a/c/d` | Visual projections that detect a small moving object (courtship, pursuit) |
| | `LC9`, `LC31a` | Visual projections associated with walking forward |
| | `LPLC2`, `LC4` | *Looming* detectors: something growing fast on the retina (an approaching predator) |
| **Output** | `DNp01` (**Giant Fiber**) | The thickest neuron: triggers the escape jump |
| | `DNp09` (**P9**) | Walk forward |
| | `DNa02` | Turn towards its side |
| | `MDN` (*moonwalker*) | Walk backwards |

**Descending neurons** (*DN*) run from the brain down to the ventral nerve cord,
which is what moves the fly's legs. They are the natural place to "plug in" the
robot's body. How these families were chosen is covered in
[the pilot](07-pilot.md) and in [tools](09-tools-and-glossary.md#the-673-type-sweep).

---

[← Index](README.md) · [Next: the LIF model →](02-lif-model.md)
