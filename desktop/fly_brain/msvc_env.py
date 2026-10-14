"""Prepara el compilador de C++ de Visual Studio para Brian2 en Windows.

La detección automática de setuptools no consigue lanzar cl.exe con Visual Studio
2019 (FileNotFoundError), pero funciona si el entorno de vcvarsall ya está cargado.
Esto lo carga en os.environ (lo heredan también los procesos hijos de joblib).

    import msvc_env; msvc_env.setup()   # antes de importar brian2
"""
import os
import subprocess
from pathlib import Path

VSWHERE = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / \
    "Microsoft Visual Studio" / "Installer" / "vswhere.exe"


def find_vcvarsall():
    """Ruta de vcvarsall.bat de la instalación más reciente con el compilador C++."""
    if not VSWHERE.exists():
        return None
    path = subprocess.run(
        [str(VSWHERE), "-latest", "-products", "*", "-requires",
         "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
        capture_output=True, text=True).stdout.strip()
    bat = Path(path) / "VC" / "Auxiliary" / "Build" / "vcvarsall.bat" if path else None
    return bat if bat and bat.exists() else None


def setup(arch="x64"):
    """Carga el entorno de vcvarsall en este proceso. Devuelve True si cl.exe quedó disponible."""
    if os.environ.get("DISTUTILS_USE_SDK") == "1":
        return True
    bat = find_vcvarsall()
    if bat is None:
        return False
    out = subprocess.run(f'"{bat}" {arch} >nul && set', shell=True, capture_output=True,
                         text=True, encoding="mbcs", errors="replace").stdout
    env = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
    if not env.get("VCToolsInstallDir"):
        return False
    os.environ.update(env)
    # Usar el entorno ya cargado en vez de volver a buscar Visual Studio
    os.environ["DISTUTILS_USE_SDK"] = "1"
    os.environ["MSSdk"] = "1"
    return True
