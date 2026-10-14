"""Descarga los datos públicos de FlyWire (~130 MB) en fly_brain/data/.

    python download_data.py
"""
import urllib.request

from connectome import DATA

FILES = {
    # Shiu et al. 2024 (MIT): conectividad y lista de neuronas de FlyWire v783
    'Connectivity_783.parquet':
        'https://github.com/philshiu/Drosophila_brain_model/raw/main/Connectivity_783.parquet',
    'Completeness_783.csv':
        'https://github.com/philshiu/Drosophila_brain_model/raw/main/Completeness_783.csv',
    # Schlegel et al. 2024: tipo celular, clase y lado de cada neurona
    'neuron_annotations.tsv':
        'https://github.com/flyconnectome/flywire_annotations/raw/main/'
        'supplemental_files/Supplemental_file1_neuron_annotations.tsv',
}


def main():
    DATA.mkdir(exist_ok=True)
    for name, url in FILES.items():
        path = DATA / name
        if path.exists():
            print(f"✓ {name} ya existe")
            continue
        print(f"↓ {name} …", flush=True)
        urllib.request.urlretrieve(url, path)
        print(f"  {path.stat().st_size / 1e6:.1f} MB")


if __name__ == '__main__':
    main()
