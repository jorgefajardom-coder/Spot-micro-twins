"""Cuevas reales escaneadas con LiDAR: KarstConduitCatalogue (Racine, Trunz, Renard y
Straubhaar, 2025, ERC Karst). Se descargan del visor Potree público
(github.com/tr1813/karstconduitcatalogue-potree, CC BY-NC 4.0; dataset completo en
https://doi.org/10.60544/7rk6-rn14) y no se guardan en el repositorio.

    python caves/real_cave.py GrotteDeLaSourde/EntranceGallery

Formato Potree 2.0 sin comprimir: octree.bin son los puntos uno detrás de otro con los
atributos de metadata.json; position es int32 * scale + offset.
"""
import json
from pathlib import Path
import sys
import urllib.request

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = "https://raw.githubusercontent.com/tr1813/karstconduitcatalogue-potree/main/potree/"
TYPES = {"int32": "<i4", "uint16": "<u2", "uint8": "u1", "double": "<f8", "int8": "i1", "float": "<f4"}
ATTRIBUTION = ("Cueva real: KarstConduitCatalogue, Racine et al. 2025 (ERC Karst), "
               "doi:10.60544/7rk6-rn14, CC BY-NC 4.0")


def download(name):
    """Descarga metadata.json y octree.bin de una galería (si no están ya)."""
    folder = HERE / "karst" / name.replace("/", "_")
    folder.mkdir(parents=True, exist_ok=True)
    for f in ("metadata.json", "octree.bin"):
        if not (folder / f).exists():
            urllib.request.urlretrieve(BASE + name + "/" + f, folder / f)
    return folder


def load_points(name):
    """Puntos (N x 3, metros, z arriba), colores RGB (0-255) y normales de una galería."""
    folder = download(name)
    meta = json.loads((folder / "metadata.json").read_text())
    fields = []
    for a in meta["attributes"]:
        base = TYPES[a["type"]]
        n = a["size"] // np.dtype(base).itemsize
        fields.append((a["name"], base, (n,)) if n > 1 else (a["name"], base))
    raw = np.fromfile(folder / "octree.bin", dtype=np.dtype(fields), count=meta["points"])
    pts = raw["position"] * np.array(meta["scale"]) + np.array(meta["offset"])
    rgb = (raw["rgb"] / 256).astype(np.uint8) if "rgb" in raw.dtype.names else None
    names = raw.dtype.names
    normals = (np.column_stack([raw["NormalX"], raw["NormalY"], raw["NormalZ"]])
               if "NormalZ" in names else None)
    return pts, rgb, normals


if __name__ == "__main__":
    for name in sys.argv[1:]:
        p, c, _ = load_points(name)
        print(name, len(p), "puntos", np.ptp(p, axis=0).round(1), "m")
