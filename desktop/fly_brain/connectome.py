"""Conectoma de FlyWire (v783) listo para simular.

Lee los datos públicos y los convierte en una matriz dispersa de pesos más una
tabla de anotaciones para buscar neuronas por tipo celular (p. ej. 'DNp01').

Datos (en fly_brain/data/, ver download_data.py):
- Connectivity_783.parquet, Completeness_783.csv
  Shiu et al. 2024, github.com/philshiu/Drosophila_brain_model (MIT)
- neuron_annotations.tsv
  Schlegel et al. 2024, github.com/flyconnectome/flywire_annotations
"""
from pathlib import Path

import numpy as np
import pandas as pd
import scipy.sparse as sp

DATA = Path(__file__).resolve().parent / "data"
CACHE = DATA / "connectome_cache.npz"


class Connectome:
    def __init__(self, weights, root_ids, annotations):
        self.weights = weights            # CSC (post x pre), sinapsis con signo
        self.root_ids = root_ids          # índice del simulador -> root_id de FlyWire
        self.annotations = annotations    # una fila por neurona, en el orden del simulador

    @property
    def n(self):
        return len(self.root_ids)

    @classmethod
    def load(cls, data=DATA):
        ann = pd.read_csv(data / "neuron_annotations.tsv", sep="\t", low_memory=False,
                          usecols=["root_id", "super_class", "cell_class", "cell_type",
                                   "hemibrain_type", "side", "top_nt"])
        if CACHE.exists() and data == DATA:
            z = np.load(CACHE)
            weights = sp.csc_matrix((z["data"], z["indices"], z["indptr"]), shape=tuple(z["shape"]))
            root_ids = z["root_ids"]
        else:
            root_ids = pd.read_csv(data / "Completeness_783.csv", index_col=0).index.to_numpy()
            con = pd.read_parquet(data / "Connectivity_783.parquet",
                                  columns=["Presynaptic_Index", "Postsynaptic_Index",
                                           "Excitatory x Connectivity"])
            n = len(root_ids)
            weights = sp.csc_matrix(
                (con["Excitatory x Connectivity"].to_numpy(np.float32),
                 (con["Postsynaptic_Index"].to_numpy(), con["Presynaptic_Index"].to_numpy())),
                shape=(n, n))
            weights.sum_duplicates()
            if data == DATA:
                np.savez(CACHE, data=weights.data, indices=weights.indices, indptr=weights.indptr,
                         shape=np.array(weights.shape), root_ids=root_ids)
        ann = ann.drop_duplicates("root_id").set_index("root_id").reindex(root_ids)
        return cls(weights, root_ids, ann.reset_index())

    def find(self, cell_type, side=None):
        """Índices de las neuronas de un tipo celular (busca también hemibrain_type)."""
        a = self.annotations
        mask = (a.cell_type == cell_type) | (a.hemibrain_type == cell_type)
        if side:
            mask &= a.side == side
        return np.flatnonzero(mask.to_numpy())

    def find_prefix(self, prefix, side=None):
        a = self.annotations
        mask = a.cell_type.fillna("").str.startswith(prefix)
        if side:
            mask &= a.side == side
        return np.flatnonzero(mask.to_numpy())
