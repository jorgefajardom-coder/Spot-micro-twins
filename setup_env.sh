#!/usr/bin/env sh
# Crea el entorno virtual del proyecto en .venv (no toca las librerías globales)
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r desktop/requirements.txt
echo "Listo. Para usarlo:  source .venv/bin/activate"
