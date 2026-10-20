#!/usr/bin/env python
# encoding: utf-8
import tkinter as tk
from tkinter import ttk
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import get_closest_to
from artelib.rotationmatrix import Rx, Ry, Rz
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.simulation import Simulation

DELTA=0.05
DELTA_CARTESIAN=0.05

class ControlRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Move robot (6 axes)")
        self.root.geometry("520x680")
        self.root.resizable(False, False)
        # Variables para almacenar el valor de cada articulación (-180° a 180°)
        self.valores_q = [tk.DoubleVar(value=0.0) for _ in range(6)]
        self.open_close = False
        self.precision = 1.0

        self.simulation = Simulation()
        self.simulation.start()
        self.robot = RobotABBIRB140(simulation=self.simulation)
        self.robot.start()
        self.gripper = GripperRG2(simulation=self.simulation)
        self.gripper.start()

        # Título
        ttk.Label(
            root,
            text="Teaching pendant",
            font=("Arial", 12, "bold")
        ).pack(pady=10)

        # Crear las filas para los 6 sliders
        for i in range(6):
            self.crear_fila_eje(i)
        self.crear_botones_cartesianos_pos()
        self.crear_botones_cartesianos_rot()
        self.crear_control_delta()
        self.rest_of_buttons()

    def crear_control_delta(self):
        """Creates the Delta increase/decrease controls at the lower part of the app."""
        frame_delta = ttk.Frame(self.root)
        frame_delta.pack(pady=5)

        btn_delta_dec = ttk.Button(
            frame_delta,
            text="Delta -",
            width=12,
            command=self.increase_precision
        )
        btn_delta_dec.pack(side="left", padx=10)

        btn_delta_inc = ttk.Button(
            frame_delta,
            text="Delta +",
            width=12,
            command=self.decrease_precision
        )
        btn_delta_inc.pack(side="left", padx=10)

    def rest_of_buttons(self):
        # --- SECCIÓN DE CONTROLES GENERALES (Reset y Pinza) ---
        frame_controles = ttk.Frame(self.root)
        frame_controles.pack(pady=15)

        # Botón de Reset
        btn_reset = ttk.Button(
            frame_controles,
            text="RESET ROBOT",
            command=self.reset_q,
            width=20
        )
        btn_reset.pack(side="left", padx=20)

        # Botón de la Pinza
        self.btn_pinza = ttk.Button(
            frame_controles,
            text="Open/close gripper ",
            command=self.toggle_gripper,
            width=20
        )
        self.btn_pinza.pack(side="left", padx=20)

    def crear_fila_eje(self, i):
        frame = ttk.Frame(self.root)
        frame.pack(fill="x", padx=15, pady=3)

        # 1. Etiqueta del nombre del eje
        label_nombre = ttk.Label(frame, text=f"q_{i+1}:", width=8)
        label_nombre.pack(side="left")

        # 2. Botón "-"
        btn_menos = ttk.Button(
            frame,
            text="-",
            width=3,
            command=lambda: self.modificar_valor(i, -DELTA)
        )
        btn_menos.pack(side="left", padx=2)

        # 3. Slider
        slider = ttk.Scale(
            frame,
            from_=-np.pi,
            to=np.pi,
            variable=self.valores_q[i],
            orient="horizontal",
            command=lambda val: self.al_cambiar_slider(i, val)
        )
        slider.pack(side="left", fill="x", expand=True, padx=5)

        # 4. Botón "+"
        btn_mas = ttk.Button(
            frame,
            text="+",
            width=3,
            command=lambda: self.modificar_valor(i, DELTA)
        )
        btn_mas.pack(side="left", padx=2)

        # 5. Etiqueta para mostrar el valor numérico
        label_valor = ttk.Label(frame, text="0.0 (rad)", width=7, anchor="e")
        label_valor.pack(side="left", padx=(5, 0))

        # Enlazar la variable con la actualización en pantalla
        self.valores_q[i].trace_add(
            "write",
            lambda *args, idx=i, lbl=label_valor: lbl.config(text=f"{self.valores_q[idx].get():.1f} (rad)")
        )

    def crear_botones_cartesianos_pos(self):
        """Crea los 6 botones cartesianos situados debajo de los sliders."""
        frame_cart = ttk.LabelFrame(self.root, text="Cartesian control (base ref. system)")
        frame_cart.pack(fill="x", padx=15, pady=8)

        # Frame contenedor para los botones
        frame_botones = ttk.Frame(frame_cart)
        frame_botones.pack(pady=5)

        # Estilo opcional para ajustar el tamaño/padding de los botones cartesianos
        style = ttk.Style()
        style.configure("Medium.TButton", font=("Arial", 10, "bold"), padding=4)

        botones = [
            ("X+", "X", DELTA_CARTESIAN),
            ("X-", "X", -DELTA_CARTESIAN),
            ("Y+", "Y", DELTA_CARTESIAN),
            ("Y-", "Y", -DELTA_CARTESIAN),
            ("Z+", "Z", DELTA_CARTESIAN),
            ("Z-", "Z", -DELTA_CARTESIAN),
        ]

        # Distribución en 2 filas de 3 botones
        for idx, (label, eje, paso) in enumerate(botones):
            row = idx // 2
            col = idx % 2
            btn = ttk.Button(
                frame_botones,
                text=label,
                width=8,
                style="Medium.TButton",
                command=lambda e=eje, p=paso: self.mover_cartesiano_pos(e, p)
            )
            btn.grid(row=row, column=col, padx=8, pady=3)

    def crear_botones_cartesianos_rot(self):
        """Crea los 6 botones para rotación."""
        frame_cart = ttk.LabelFrame(self.root, text="Rotation control (end effector's ref. system)")
        frame_cart.pack(fill="x", padx=15, pady=8)

        # Frame contenedor para los botones
        frame_botones = ttk.Frame(frame_cart)
        frame_botones.pack(pady=5)

        # Estilo opcional para ajustar el tamaño/padding de los botones cartesianos
        style = ttk.Style()
        style.configure("Medium.TButton", font=("Arial", 10, "bold"), padding=4)

        botones = [
            ("X+", "X", DELTA_CARTESIAN),
            ("X-", "X", -DELTA_CARTESIAN),
            ("Y+", "Y", DELTA_CARTESIAN),
            ("Y-", "Y", -DELTA_CARTESIAN),
            ("Z+", "Z", DELTA_CARTESIAN),
            ("Z-", "Z", -DELTA_CARTESIAN),
        ]

        # Distribución en 2 filas de 3 botones
        for idx, (label, eje, paso) in enumerate(botones):
            row = idx // 2
            col = idx % 2
            btn = ttk.Button(
                frame_botones,
                text=label,
                width=8,
                style="Medium.TButton",
                command=lambda e=eje, p=paso: self.mover_cartesiano_rot(e, p)
            )
            btn.grid(row=row, column=col, padx=8, pady=3)

    def mover_cartesiano_pos(self, eje, paso):
        """Función callback para mover el robot en coordenadas cartesianas X, Y, Z."""
        print(f"Moviendo cartesiano Eje {eje} en paso {paso*self.precision}")
        q0 = []
        for i in range(6):
            val = self.valores_q[i].get()
            q0.append(val)
        q0 = np.array(q0)

        T = self.robot.directkinematics(q0)
        if eje=='X':
            v = np.array([paso*self.precision, 0, 0])
        elif eje=='Y':
            v = np.array([0, paso*self.precision, 0])
        else:
            v = np.array([0, 0, paso*self.precision])
        # modificar T.pos() con 'paso' en el 'eje' indicado
        T = HomogeneousMatrix(T.pos()+v, T.R())
        q_is = self.robot.inverse_kinematics(target_position=T.pos(),
                                                 target_orientation=T.R())
        # find the solutions that is closest to the previous solution
        q_target = get_closest_to(q0, q_is)
        # actualizar self.valores_q y mover_robot()
        for i in range(6):
            self.valores_q[i].set(q_target[i])
        self.mover_robot()

    def mover_cartesiano_rot(self, eje, paso):
        """Función callback para mover el robot en rotaciones X, Y, Z."""
        print(f"Girando en Eje {eje} en paso {paso*self.precision}")
        q0 = []
        for i in range(6):
            val = self.valores_q[i].get()
            q0.append(val)
        q0 = np.array(q0)

        T = self.robot.directkinematics(q0)
        if eje=='X':
            R = Rx(paso*self.precision)
        elif eje=='Y':
            R = Ry(paso*self.precision)
        else:
            R = Rz(paso*self.precision)
        # modificar la rotación y se crea una nueva matriz T
        T = HomogeneousMatrix(T.pos(), T.R()*R)
        q_is = self.robot.inverse_kinematics(target_position=T.pos(),
                                            target_orientation=T.R())
        # find the solutions that is closest to the previous solution
        q_target = get_closest_to(q0, q_is)
        # actualizar self.valores_q y mover_robot()
        for i in range(6):
            self.valores_q[i].set(q_target[i])
        self.mover_robot()

    def reset_q(self):
        """Resetea las articulaciones"""
        for i in range(6):
            self.valores_q[i].set(0)
        self.mover_robot()

    def toggle_gripper(self):
        self.open_close = not self.open_close
        if self.open_close:
            self.gripper.open()
        else:
            self.gripper.close()
        self.robot.wait(10)

    def increase_precision(self):
        self.precision=self.precision/2

    def decrease_precision(self):
        self.precision=self.precision*2

    def modificar_valor(self, indice, cambio):
        """Aumenta o disminuye el valor actual al pulsar + o -"""
        valor_actual = self.valores_q[indice].get()
        nuevo_valor = max(-np.pi, min(np.pi, valor_actual + cambio))
        self.valores_q[indice].set(nuevo_valor)
        self.mover_robot()

    def al_cambiar_slider(self, indice, valor):
        """Callback ejecutado al arrastrar el slider"""
        val_float = float(valor)
        self.valores_q[indice].set(val_float)
        self.mover_robot()

    def mover_robot(self):
        """Punto de integración para enviar datos al robot (Puerto Serie, ROS, Wifi, etc.)"""
        # print(f"Moviendo Eje {eje + 1} -> {valor:.1f}°")
        q = []
        for qi in self.valores_q:
            q.append(float(qi.get()))
        self.robot.moveAbsJ(q_target=q, precision=False, speed_factor=10.0)
        T = self.robot.directkinematics(q)
        Q = T.Q()
        print('Current q is: \n', q)
        print('End effector T is: \n')
        T.print_nice()
        print('End effector position is (p): \n', T.pos())
        print('End effector orientation is (alpha, beta, gamma): \n', T.euler()[0])
        print('End effector Q is (Quaternion): ', Q)

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlRobotApp(root)
    root.mainloop()