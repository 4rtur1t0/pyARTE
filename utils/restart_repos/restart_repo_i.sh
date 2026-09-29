#!/bin/bash

address=$1

# --- Configuración ---
SERVIDOR="172.16.28.$address"
USUARIO="usuario"
CONTRASENA=usuario

echo "INTENTANDO CONECTAR con IP" $SERVIDOR
# POR SIMPLICIDAD, SALIMOS SI EL SERVIDOR NO RESPONDE
nc -z -w 5 "$SERVIDOR" 22 || exit 1

echo "=== Conectando a $SERVIDOR mediante contraseña y resetando repositorios ==="

# Usamos sshpass para enviar la contraseña de forma no interactiva
sshpass -p "$CONTRASENA" ssh -o StrictHostKeyChecking=no "$USUARIO@$SERVIDOR" "
    # repositories
    echo "ELIMINANDO Y CLONANDO ARTE"
    cd ~/Escritorio/
    rm -R ARTE
    git clone https://github.com/4rtur1t0/ARTE

    echo "ELIMINANDO Y CLONANDO PYARTE"
    cd ~/Escritorio/
    rm -R pyARTE
    git clone https://github.com/4rtur1t0/pyARTE

    echo "ELIMINANDO Y CLONANDO INTRO2AI"
    cd ~/Escritorio/
    rm -R Intro2AI
    git clone https://github.com/4rtur1t0/Intro2AI

    echo "Instalacion completada con exito"
"

echo "=== Proceso finalizado ==="

