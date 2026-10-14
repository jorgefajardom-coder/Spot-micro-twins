"""Vista del robot compartida por los controles inversos y diferenciales."""
import numpy as np

from spot_model3d import SpotModel, SpotScene, solve_skeleton


class RobotViewport:
    def __init__(self, ax, kin):
        self.ax, self.kin = ax, kin
        self.mode = 'Ambos'
        self.zoom = 1.1
        self.pose = None
        self.markers = []
        self.scene = SpotScene(ax, SpotModel(kin), show_skeleton=True)
        ax.figure.set_facecolor('#0e1218')
        ax.set_axis_off()
        ax.view_init(elev=23, azim=-32)
        # Esqueleto superpuesto para inspeccionar articulaciones dentro de la carcasa.
        ax.computed_zorder = False
        self.scene.mesh.set_zorder(2)
        self.scene.shadows.set_zorder(1)
        ax.figure.canvas.mpl_connect('scroll_event', self.scroll)

    def draw(self, feet, rot, body, is_radians=False, limit=0.25):
        self.pose = (np.array(feet, copy=True), np.array(rot, copy=True),
                     np.array(body, copy=True), is_radians, limit)
        skel = solve_skeleton(self.kin, feet, rot, body, is_radians)
        for marker in self.markers:
            marker.remove()
        self.markers.clear()
        self.scene.show_model = self.mode in ('Modelo', 'Ambos')
        self.scene.show_skeleton = self.mode in ('Esqueleto', 'Ambos')
        self.scene.draw([(skel, (0, 0, 0), None)], span=limit, zoom=self.zoom)
        for line in self.scene.lines:
            # Colocar la cuadrícula detrás y las articulaciones delante del CAD.
            line.set_zorder(5 if line.get_linewidth() > 1 else 0)
        errors = [i for i, leg in enumerate(skel['legs']) if leg is None]
        for leg in errors:
            p = feet[leg]
            self.markers.append(self.ax.scatter(*p, color='#ff6464', marker='X', s=75, zorder=6))
        return skel['angles'], errors

    def redraw(self):
        """Cambios de cámara o capas: solo dibujar, nunca enviar al robot."""
        if self.pose is not None:
            self.draw(*self.pose)
        self.ax.figure.canvas.draw_idle()

    def set_mode(self, mode):
        if mode not in ('Esqueleto', 'Modelo', 'Ambos'):
            raise ValueError('Modo de vista desconocido')
        self.mode = mode
        self.redraw()

    def set_camera(self, name):
        views = {'Perspectiva': (23, -32), 'Lateral': (0, -90), 'Superior': (90, -90)}
        elev, azim = views[name]
        self.ax.view_init(elev=elev, azim=azim)
        self.redraw()

    def scroll(self, event):
        if event.inaxes is self.ax:
            self.zoom = float(np.clip(self.zoom * 1.08 ** event.step, 0.7, 1.8))
            self.redraw()

    def add_controls(self, parent):
        import tkinter as tk
        from tkinter import ttk
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=4)
        ttk.Label(frame, text='Vista:').grid(row=0, column=0, padx=3)
        mode = ttk.Combobox(frame, values=('Esqueleto', 'Modelo', 'Ambos'),
                            state='readonly', width=10)
        mode.set(self.mode)
        mode.grid(row=0, column=1, padx=3)
        mode.bind('<<ComboboxSelected>>', lambda _: self.set_mode(mode.get()))
        camera = ttk.Combobox(frame, values=('Perspectiva', 'Lateral', 'Superior'),
                              state='readonly', width=12)
        camera.set('Perspectiva')
        camera.grid(row=0, column=2, padx=3)
        camera.bind('<<ComboboxSelected>>', lambda _: self.set_camera(camera.get()))
        floor = tk.BooleanVar(value=True)
        def toggle_floor():
            self.scene.show_floor = floor.get()
            self.redraw()
        ttk.Checkbutton(frame, text='Suelo', variable=floor,
                        command=toggle_floor).grid(row=1, column=0, columnspan=2, sticky='w')
        ttk.Label(frame, text='Arrastra: girar · Rueda: zoom').grid(row=1, column=2, sticky='w')
        self.controls = (mode, camera, floor)
