
# configurese con los datos
COPPELIA_PATH=~/Aplicaciones/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/
PYCHARM_PATH=~/Aplicaciones/pycharm-2026.2.0.1/

DESKTOP_COPPELIA_FILE=~/Escritorio/coppeliasim.desktop
DESKTOP_MATLAB_FILE=~/Escritorio/matlab.desktop
DESKTOP_PYCHARM_FILE=~/Escritorio/pycharm.desktop

address=$1

#!/bin/bash

# --- Configuración ---
SERVIDOR="172.16.28.$address"
USUARIO="usuario"
CONTRASENA=usuario
LIBRERIAS="git python3 python3-virtualenv"

# POR SIMPLICIDAD, SALIMOS SI EL SERVIDOR NO RESPONDE
nc -z -w 5 "$SERVIDOR" 22 || exit 1

echo "=== Conectando a $SERVIDOR mediante contraseña e instalando librerías ==="

# Usamos sshpass para enviar la contraseña de forma no interactiva
sshpass -p "$CONTRASENA" ssh -o StrictHostKeyChecking=no "$USUARIO@$SERVIDOR" "
    echo "Actualizando lista de paquetes..."
    echo "$CONTRASENA" | sudo -S apt update -y
    echo "$CONTRASENA" | sudo -S apt install -y git python3 python3-virtualenv
    echo "$CONTRASENA" | sudo -S apt install -y libavcodec-dev libavformat-dev libswscale-dev
    echo "$CONTRASENA" | sudo -S apt install -y net-tools

    # repositories
    echo "CLONANDO ARTE"
    cd ~/Escritorio/
    git clone https://github.com/4rtur1t0/ARTE

    echo "CLONANDO PYARTE"
    cd ~/Escritorio/
    git clone https://github.com/4rtur1t0/pyARTE

    echo "INSTALANDO VIRTUALENV Y LIBRERÍAS PARA PYARTE"
    # install virtualenv and python libraries
    cd ~/Escritorio/
    virtualenv venv
    cd ~/Escritorio/venv/bin
    ./pip install -r ../../pyARTE/requirements.txt

    echo "CLONANDO INTRO2AI"
    cd ~/Escritorio/
    git clone https://github.com/4rtur1t0/Intro2AI

    echo "INSTALANDO VIRTUALENV Y LIBRERÍAS PARA INTRO2AI"
    # install virtualenv and python libraries
    cd ~/Escritorio/
    virtualenv venvAI
    cd ~/Escritorio/venvAI/bin
    ./pip install -r ../../Intro2AI/requirements.txt

    mkdir /home/usuario/Aplicaciones

    echo "Instalacion completada con exito"
"

# TRANSFERIMOS COPPELIA SIM
# CUIDADO, el directorio Aplicaciones debe existir
echo "Transferring Coppelia Sim"
sshpass -p usuario scp -r -O "$COPPELIA_PATH" usuario@172.16.28.$address:"$COPPELIA_PATH"
# TRANSFERENCIA DE PYCHARM
echo "Transferring pyCharm"
sshpass -p usuario scp -r -O "$PYCHARM_PATH" usuario@172.16.28.$address:"$PYCHARM_PATH"
# transferimos iconos
echo "Transferring Desktop icons"
sshpass -p usuario scp -r $DESKTOP_COPPELIA_FILE usuario@172.16.28.$address:~/Escritorio
sshpass -p usuario scp -r $DESKTOP_MATLAB_FILE usuario@172.16.28.$address:~/Escritorio
sshpass -p usuario scp -r $DESKTOP_PYCHARM_FILE usuario@172.16.28.$address:~/Escritorio


echo "=== Proceso finalizado ==="

