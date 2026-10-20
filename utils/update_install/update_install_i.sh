#!/bin/bash
# configurese con los datos
#COPPELIA_PATH=~/Aplicaciones/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/
#PYCHARM_PATH=~/Aplicaciones/pycharm-2026.2.0.1/

address=$1

# --- Configuración ---
SERVIDOR="172.16.28.$address"
USUARIO="usuario"
CONTRASENA=usuario
LIBRERIAS="git python3 python3-virtualenv"

echo "INTENTANDO CONECTAR con IP" $SERVIDOR
# POR SIMPLICIDAD, SALIMOS SI EL SERVIDOR NO RESPONDE
nc -z -w 5 "$SERVIDOR" 22 || exit 1

echo "=== Conectando a $SERVIDOR mediante contraseña e instalando librerías ==="

# Usamos sshpass para enviar la contraseña de forma no interactiva
sshpass -p "$CONTRASENA" ssh -o StrictHostKeyChecking=no "$USUARIO@$SERVIDOR" "
    echo "$CONTRASENA" | sudo -S apt install -y python3-tk

    # repositories
    #echo "CLONANDO ARTE"
    #cd ~/Escritorio/
    #git clone https://github.com/4rtur1t0/ARTE

    echo "Instalacion completada con exito"
"

# EJEMPLO DE TRANSFERENCIA DE FICHERO
# CUIDADO, el directorio Aplicaciones debe existir
#echo "Transferring Coppelia Sim"
#sshpass -p usuario scp -r -O "$COPPELIA_PATH" usuario@172.16.28.$address:"$COPPELIA_PATH"

echo "=== Proceso finalizado ==="

