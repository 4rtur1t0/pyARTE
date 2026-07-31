!/bin/bash
address=$1


# --- Configuración ---
SERVIDOR="172.16.28.$address"
USUARIO="usuario"
CONTRASENA=usuario


# POR SIMPLICIDAD, SALIMOS SI EL SERVIDOR NO RESPONDE
nc -z -w 5 "$SERVIDOR" 22 || exit 1

echo "=== Conectando a $SERVIDOR mediante contraseña e instalando librerías ==="

# TRANSFERIMOS COPPELIA SIM
echo "Transferring Matlab"
# ojo, se transfiere a Aplicaciones
sshpass -p usuario scp -r /usr/local/MATLAB/* usuario@172.16.28.$address:/home/usuario/Aplicaciones/MATLAB

#now move to /usr/local/MATLAB
sshpass -p "$CONTRASENA" ssh -o StrictHostKeyChecking=no "$USUARIO@$SERVIDOR" "
    echo "Moviendo Matlab a /usr/local..."
    echo "$CONTRASENA" | sudo -S mv /home/usuario/Aplicaciones/MATLAB /usr/local/MATLAB
    echo "Instalacion completada con exito"
"