#!/usr/bin/env python
# encoding: utf-8
import tkinter as tk
from tkinter import ttk
import numpy as np

from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.simulation import Simulation

DELTA=0.05

class ControlRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Move robot (6 axes)")
        self.root.geometry("520x320")
        self.root.resizable(False, False)
        # Variables para almacenar el valor de cada articulación (-180° a 180°)
        self.valores_q = [tk.DoubleVar(value=0.0) for _ in range(6)]
        self.open_close = False

        simulation = Simulation()
        simulation.start()
        self.robot = RobotABBIRB140(simulation=simulation)
        self.robot.start()
        self.gripper = GripperRG2(simulation=simulation)
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
        self.rest_of_buttons()

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
        print('End effector T is: \n', T)
        print('End effector position is (p): \n', T.pos())
        print('End effector orientation is (alpha, beta, gamma): \n', T.euler()[0])
        print('End effector Q is (Quaternion): ', Q)

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlRobotApp(root)
    root.mainloop()