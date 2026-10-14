# fly_brain: el cerebro de la mosca para el Spot Micro

Simulación del cerebro completo de *Drosophila* (FlyWire v783: 138.639 neuronas,
15 millones de conexiones) con el modelo LIF de Shiu et al. (*Nature*, 2024).
La idea es que la mosca **decida** (avanzar, girar, huir, retroceder) y que
`gait.py` sea la médula espinal que lo convierte en pasos del robot.

## Instalación

Usa su propio entorno virtual, separado del de la GUI:

```bash
cd desktop/fly_brain
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\pip install "cupy-cuda12x[ctk]"   # opcional: GPU NVIDIA (trae CUDA, sin toolkit)
.venv\Scripts\python download_data.py
```

Si hay GPU CUDA se usa sola; `--device cpu` fuerza la CPU.

Opcional, para ver la forma real de cada neurona en el visor (~1 h de descarga,
reanudable; se guarda simplificada en `data/skeletons_lite.npz`):

```bash
.venv\Scripts\python download_skeletons.py
```

Los esqueletos son los precomputados de FlyWire 783 (grupo de Jefferis, MRC LMB;
los mismos que `fafbseg.flywire.get_skeletons`, CC BY 4.0). Cada uno se reduce
agrupando vértices en celdas de 3 µm al descargar y de 12 µm para el visor.

## Uso

```bash
python probe.py LPLC2 LC4                # looming en ambos ojos -> Giant Fiber
python probe.py LC10a --side left        # objeto a la izquierda -> DNa02 izquierda
python probe.py --scan                   # todos los tipos sensoriales -> data/scan.csv
python validate_brian2.py                # comparar con el modelo original (Brian2)
python -m unittest discover -s tests -v
python ..\spot_brain.py                  # el Spot manejado por la mosca, con el cerebro en 3D
python ..\spot_brain.py --record spot.mp4  # grabarlo en vídeo
```

![El Spot persiguiendo un objetivo y huyendo de un obstáculo](../../docs/assets/spot_brain.gif)

La interfaz está en inglés. A la izquierda, grande, el robot (el CAD completo,
699.000 caras) en la cueva. A la derecha, dos pantallas a la vez en tres pestañas
(botones o teclas `1`-`3`): **Brain · LiDAR map**, **Eyes · Descending** e
**History · Sensors**.

El cerebro tiene la morfología real de cada neurona de FlyWire, en vista frontal
fija que llena la pantalla (se gira con el ratón), coloreada como las
renderizaciones de FlyWire: un color por tipo celular, cian/turquesa/amarillo en
los lóbulos ópticos y violeta/magenta/azul en el cerebro central. La actividad va
en una capa encima con la misma cámara: las neuronas que disparan brillan en
naranja dentro del cerebro, y sus sinapsis más fuertes salen del centro de su
morfología (naranja excitatoria, azul inhibitoria). Las demás pantallas muestran el mapa LiDAR, los estímulos visuales,
las descendentes, los últimos segundos de avance, giro y Giant Fiber, y los
sensores de gas y temperatura. Render con PyVista/VTK (OpenGL con z-buffer, FXAA y
oclusión ambiental).

### Brazo con cabeza y LiDAR

El Spot lleva un brazo al estilo del Spot Arm de Boston Dynamics, con sus
proporciones (brazo 338,5 mm, antebrazo 403,3, muñeca 117,45, pinza 124,82)
escaladas ×0,28 al Spot Micro (`../spot_arm.py`): base negra que gira, hombro,
brazo amarillo, codo con su tramo negro doblado, antebrazo, muñeca y una cabeza con
pinza y cámara. El LiDAR va encima de la cabeza (~0,37 m sobre el suelo) y la
cabeza **mira alrededor**: la base barre ±65° cada 9 s y la muñeca la mantiene
nivelada con un leve cabeceo; el LiDAR escanea desde donde está la cabeza en cada
momento.

## Misión en la cueva: gas, calor, terreno y LiDAR

`../sim_world.py` simula la cueva (sin gráficos, con tests). Por defecto es una
**cueva real**: la galería de entrada de la Grotte de la Sourde, escaneada con
LiDAR (KarstConduitCatalogue, Racine et al. 2025, ERC Karst,
[doi:10.60544/7rk6-rn14](https://doi.org/10.60544/7rk6-rn14), CC BY-NC 4.0),
a escala 2:1: una **galería de mina por la que pasan personas** (~2,7 m de alto y
1,2-1,8 m de ancho), con la fuga a 14 m de la
entrada. `../caves/real_cave.py` descarga la nube de
puntos (Potree) la primera vez; de sus normales salen el suelo y el techo, la
zona transitable y la ruta por el centro de la galería. Las paredes se pintan con
la luminosidad del escáner sobre un tono de caliza, sedimento en el suelo y grano
de roca. Con `--cueva generada` se usa la galería sintética de 16 m.

- **Fuga de gas** al fondo: el gas se difunde a lo largo de la galería. Dos
  "antenas" virtuales miden el gradiente; el robot lo sigue hasta la fuga y se
  para a distancia segura (≥ 450 ppm), que queda marcada en el mapa.
- **Fuente de calor** en una sala: desde 28 °C el robot la rodea por el lado
  contrario; desde 48 °C además entra como amenaza y la Giant Fiber lo hace huir.
- **Terreno escarpado** (botón o tecla `t`): cada pie apoya a la altura real del
  suelo y el cuerpo se inclina con la pendiente (cabeceo y alabeo).
- **LiDAR 360°** (96 azimuts × 18 elevaciones de −84° a +90°: suelo, paredes y
  techo; 6 m, trazado con numba sobre una rejilla de la cueva): las paredes
  cercanas lo desvían hacia el lado despejado y cada barrido se acumula en un mapa
  de vóxeles, que el panel del mapa dibuja sin techo para ver la galería.

En la prueba sin gráficos el cerebro completo encuentra la fuga en ~52 s en la
cueva generada y en ~50 s en la galería de mina, en terreno plano y escarpado, sin
chocar.

La roca es rugosa (ruido 3D que escala la sección de la galería; el LiDAR y la
malla usan exactamente la misma deformación). El mapa no es una nube de puntos:
los vóxeles del LiDAR se convierten en una rejilla de ocupación, se suavizan y se
extrae la superficie (*flying edges* + Taubin), coloreada por altura como un
escaneo real.

### Detener el escaneo y explorar la cueva

El botón **"Stop scan and build the 3D cave"** (panel del robot) para al
robot, reconstruye la cueva entera (con techo) y la abre en `../cave_explorer.py`:

- capas **Rock / Gas / Temperature**, un deslizador para cortar el techo (hasta la
  altura total de la cueva) y el botón **Closed cave** para verla cerrada, con
  su techo escaneado;
- el gas y la temperatura de cada punto de la roca salen de las **mediciones
  reales de los sensores** a lo largo de la ruta (media ponderada por distancia,
  hasta 1,1 m); donde el robot no midió queda en gris, "sin datos";
- la ruta coloreada por lo medido, la fuga y el punto más caliente marcados;
- se guarda en `desktop/scans/` como `.vtp` (todas las capas), `.json`
  (registro de sensores) y `.ply` con colores para abrirlo en Blender o MeshLab.

```bash
python ..\cave_explorer.py ..\scans\cueva_AAAAMMDD_HHMMSS.vtp
```

**Por qué el gas y el calor entran por circuitos visuales.** En este modelo las
vías olfativa y térmica no sirven para orientarse: estimular 13 neuronas de calor
a 5 Hz ya desata ~150.000 espigas por todo el cerebro, y las olfativas igual con
cualquier intensidad; la salida es siempre la misma (DNa02 izquierda) venga el
estímulo de un lado o del otro. El lóbulo antenal es muy recurrente y el modelo
no tiene adaptación. Por eso se usa **sustitución sensorial**: el gradiente de
gas entra como "objetivo" por LC10/LC9 (persecución, validado) y el calor fuerte
como *looming* por LPLC2/LC4 (amenaza, validado). Mejorar el modelo olfativo
(inhibición de los LN, adaptación de los ORN) sería el paso para usar el olfato
real de la mosca.

## Piloto: de lo que ve el robot a la marcha

`pilot.py` (`FlyPilot`) estimula las neuronas visuales según lo que percibe el
robot, corre el cerebro y traduce las descendentes a `vx` y giro para
`gait.GaitManager.set_command`:

| Percepción | Neuronas estimuladas | Descendentes que se leen | Comando |
|---|---|---|---|
| objetivo a un lado | LC10 de ese lado | DNa02 izq./dcha. | giro hacia él |
| objetivo delante | LC9 + LC31a | DNp09 (P9) | avanzar |
| algo se acerca | LPLC2 + LC4 | Giant Fiber, MDN | escape: retroceder |

El conectoma no es simétrico, así que `calibrate()` mide la respuesta máxima de
cada lado al arrancar y normaliza. Una salida suavizada (150 ms) evita tirones
en los servos.

```python
from connectome import Connectome
from lif import Params, make_brain

c = Connectome.load()
brain = make_brain(c, Params(dt=0.1))   # GPU si hay CUDA, si no CPU
brain.stimulate((c.find('LPLC2'), 150))  # 150 Hz, como la optogenética del artículo
counts = brain.run(100)                  # espigas por neurona en 100 ms
print(brain.rate(counts, 100, c.find('DNp01')))
```

## Simulador

Reproduce el `model.py` original (Brian2): mismas ecuaciones y parámetros,
integración exacta (`method='linear'`) y el mismo orden por paso (integrar → umbral
→ sinapsis → reset). Está reescrito para poder avanzar a trozos desde el bucle del
robot y para usar la GPU. Dos implementaciones con la misma API y los mismos tests:

- `lif_gpu.py` (CUDA vía CuPy): un hilo por neurona; las espigas se apuntan con
  `atomicAdd` y la propagación lee cuántas hay de la memoria de la GPU, sin
  sincronizar con la CPU. Los pasos se lanzan en bloques de 100 grabados como
  CUDA Graph: lanzar cada kernel desde Python costaba más que el cálculo.
- `lif.py` (CPU, numba): bucle sin ramas para que vectorice (con atajos por
  neurona era 7 veces más lento por fallos de predicción).

Velocidad con looming (LPLC2 + LC4), RTX 4060 Laptop / i7-12650H:

| | dt 0,1 ms (original) | dt 0,5 ms |
|---|---|---|
| GPU | **8,3× tiempo real** | 33× |
| CPU | 0,29× | 0,99× (3 % de diferencia en frecuencias) |

El barrido de 673 estímulos tarda 45 s en GPU con dt 0,1 ms.

### Validación contra el modelo original

`validate_brian2.py` descarga el `model.py` de Shiu et al., corre el mismo
experimento con su código (Brian2) y con el nuestro, y compara neurona a neurona.
Looming (LPLC2 + LC4, 150 Hz, 1 s):

| | original (Brian2, 8 ensayos) | nuestro (GPU, 30 ensayos) |
|---|---|---|
| tiempo | 740 s | 3,6 s |
| correlación neurona a neurona | — | **r = 0,999**, pendiente 1,003 |
| Giant Fiber (DNp01 dcha.) | 195,6 Hz | 193 Hz |

La primera versión daba r = 0,968 pero ~40 % más actividad. Las causas, aisladas con
redes de 2–3 neuronas y trazas de voltaje paso a paso:
- g es `(unless refractory)`: **Brian2 descarta la entrada sináptica que llega
  durante el refractario** (no solo congela g). Acumularla hacía que la neurona
  saliera del refractario ya cargada y volviera a disparar antes.
- Brian2 vuelve a integrar `round(t_rfc/dt)` pasos después de la espiga: congela
  21 pasos, no 22.

Brian2 en Windows necesita el compilador de Visual Studio; `msvc_env.py` carga su
entorno porque la detección automática de setuptools falla con VS 2019.

## Qué responde a qué

Barrido de los 673 tipos sensoriales/visuales por lado (150 Hz, 300 ms), mirando
las neuronas descendentes con función motora conocida:

| Estímulo | Descendente | Conducta |
|---|---|---|
| LPLC2, LC4, LC6 | DNp01 (Giant Fiber) 110–170 Hz | escape ante algo que se acerca |
| LPLC2 + LC4 de un lado | DNp02/04/11 ipsilaterales | respuesta lateralizada al looming |
| LC10a/c-2/d de un lado | DNa02 ipsilateral 77–133 Hz | perseguir un objeto (cortejo) |
| LC9, LC31a | DNp09 (P9) 50–92 Hz | caminar hacia delante |
| LPC1, LC9, LC6 | MDN 15–32 Hz | caminar hacia atrás |

## Datos

- `Connectivity_783.parquet`, `Completeness_783.csv`: Shiu et al. 2024,
  [philshiu/Drosophila_brain_model](https://github.com/philshiu/Drosophila_brain_model) (MIT).
- `neuron_annotations.tsv`: Schlegel et al. 2024,
  [flyconnectome/flywire_annotations](https://github.com/flyconnectome/flywire_annotations).
- FlyWire: Dorkenwald et al. 2024, *Nature* 634.
