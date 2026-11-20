# 12 · Solución de problemas

> 🌐 [English](../brain/12-troubleshooting.md) · **Español**

[← Añadir una conducta nueva](11-anadir-una-conducta.md) · [Índice](README.md) · [Siguiente: limitaciones y trabajo futuro →](13-limitaciones-y-trabajo-futuro.md)

---

## El cerebro no usa la GPU

Al arrancar, el visor dice dónde corre:

```
Brain on GPU · dt 0.1 ms · calibration {...}     ← bien
Brain on CPU · dt 0.5 ms · calibration {...}     ← no encontró la GPU
```

Comprueba en el entorno de `fly_brain`:

```bash
.venv\Scripts\python -c "import lif_gpu; print(lif_gpu.available())"
```

| Si sale… | Causa probable | Solución |
|---|---|---|
| `ModuleNotFoundError: cupy` | CuPy no instalado | `pip install "cupy-cuda12x[ctk]"` en el `.venv` de fly_brain |
| `False` sin error | No hay GPU NVIDIA visible, o el *driver* es antiguo | Actualiza el *driver* de NVIDIA (CUDA 12 pide un *driver* reciente); comprueba con `nvidia-smi` |
| Error al compilar el kernel | Falta el compilador de CUDA | Instala con `[ctk]` (trae NVRTC desde pip) en vez de `cupy-cuda12x` a secas |
| `CUDA path could not be detected` | Aviso de CuPy | Inofensivo con `[ctk]`; se silencia solo |

Para forzar uno u otro: `--device gpu` (falla si no hay) o `--device cpu`.

## El visor va lento

1. **Mira qué está tardando:**
   ```bash
   fly_brain\.venv\Scripts\python spot_brain.py --profile
   ```
   Al cerrar la ventana imprime la mediana y el percentil 95 de cada fase (mundo +
   LiDAR, cerebro, marcha, dibujo, render).
2. **Si "brain" es lo más caro:** el cerebro está en CPU. Mira la sección anterior.
   Sin GPU, el piloto ya usa `dt = 0,5 ms` para acercarse al tiempo real.
3. **Si "render (VTK)" es lo más caro:** casi siempre es la **gráfica integrada**. En
   Windows: *Configuración → Pantalla → Gráficos*, añade el `python.exe` del `.venv` y
   elige **Alto rendimiento** (la NVIDIA). Con la integrada el visor va a menos de la
   mitad de fps.
4. **Si es "draw":** cierra las pestañas que no mires (solo se dibujan las visibles) o
   baja `ACTIVE_SEGMENTS` / `MAX_SYNAPSE_SOURCES` en `spot_brain.py`.

## Se queda mucho rato al arrancar

- **La primera vez** numba compila las funciones (decenas de segundos) y se construye
  la caché del conectoma. Las siguientes veces es mucho más rápido.
- Si **siempre** tarda, puede que la caché de numba no se pueda escribir (carpeta de
  solo lectura) o se haya corrompido: borra las carpetas `__pycache__` de `fly_brain/`
  y vuelve a ejecutar.
- Si cambiaste los datos de FlyWire, borra `data/connectome_cache.npz` para que se
  regenere.

## Sin memoria en la GPU

Los datos del cerebro ocupan ~140 MB de GPU (pesos + estado), más unos cientos de MB del contexto de CUDA. Si falla con `OutOfMemoryError`:

- Cierra otros programas que usen la GPU (juegos, otros notebooks con CuPy/PyTorch).
- No crees varios `Brain` a la vez: usa `reset()` para repetir experimentos con el
  mismo objeto.

## No encuentra los datos

```
FileNotFoundError: .../fly_brain/data/neuron_annotations.tsv
```

Los datos no están en el repositorio (pesan ~130 MB). Descárgalos:

```bash
.venv\Scripts\python download_data.py
```

Sin `skeletons_lite.npz` el visor funciona igual, pero dibuja el cerebro como una nube
de puntos. Para la forma real: `python download_skeletons.py` (~1 h, se puede cortar y
reanudar).

## `probe.py` no encuentra un tipo celular

```
No hay neuronas de tipo ['LC10'] (ambos lados)
```

Los nombres son exactos (`LC10a`, no `LC10`). Para buscar por prefijo:

```python
c.find_prefix('LC10')
c.annotations[c.annotations.cell_type.str.startswith('LC10', na=False)].cell_type.unique()
```

## `validate_brian2.py` falla en Windows

Brian2 necesita el compilador de C++ de Visual Studio. Instala *Visual Studio Build
Tools* con el componente **"Desarrollo para el escritorio con C++"**. `msvc_env.py` lo
busca con `vswhere` y carga su entorno automáticamente.

## El robot gira siempre hacia un lado

El conectoma no es simétrico y la calibración lo compensa al arrancar. Si se ha
desajustado (p. ej. tras cambiar `INPUTS`), comprueba que `calibrate()` se llama antes
del bucle y mira las ganancias que imprime: un lado con ganancia mínima (10) suele
indicar que ese grupo no tiene neuronas en ese lado.

---

[← Añadir una conducta nueva](11-anadir-una-conducta.md) · [Índice](README.md) · [Siguiente: limitaciones y trabajo futuro →](13-limitaciones-y-trabajo-futuro.md)
