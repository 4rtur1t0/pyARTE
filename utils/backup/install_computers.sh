#
# USED TO update all computers
# Copy Coppelia Sim Directory
# Copy ARTE directory
# Copy pyARTE directory
#
# Instrucciones:
# Ordenador profesor: >> sudo apt install sshpass
# Todos ordenadores instalar: >> sudo apt install openssh-server -y
# Final: ejecutar este script ./install_computers.sh

COPPELIA_PATH=~/Applications/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/
ARTE_PATH=~/Escritorio/ARTE
pyARTE_PATH=~/Escritorio/pyARTE/

DESKTOP_COPPELIA_FILE=~/Escritorio/coppeliasim.desktop
DESKTOP_MATLAB_FILE=~/Escritorio/Matlab.desktop
DESKTOP_PYCHARM_FILE=~/Escritorio/pycharm.desktop

current_i=51

#!/bin/bash
for i in {1..100}
 do
 echo "Checking IP: 172.16.28.$i"
 if [[ $current_i -eq $i ]]; then
   echo "OWN IP"
   continue
 fi
 if ping -c1 172.16.28.$i 2>&1 2>/dev/null; then
  echo "Found up IP: 172.16.28.$i"
  echo "Transferring Coppelia Sim"
  sshpass -p usuario scp -r $COPPELIA_PATH usuario@172.16.28.$i:~/Applications
  echo "Transferring ARTE"
  sshpass -p usuario scp -r $ARTE_PATH usuario@172.16.28.$i:~/Escritorio
  echo "Transferring pyARTE"
  sshpass -p usuario scp -r $pyARTE_PATH usuario@172.16.28.$i:~/Escritorio
  echo "Transferring Desktop files"
  sshpass -p usuario scp $DESKTOP_COPPELIA_FILE usuario@172.16.28.$i:~/Escritorio
  sshpass -p usuario scp $DESKTOP_MATLAB_FILE usuario@172.16.28.$i:~/Escritorio
  sshpass -p usuario scp $DESKTOP_PYCHARM_FILE usuario@172.16.28.$i:~/Escritorio
  echo "ENDED Transfer"
 else
  echo "[ERROR] Could not connect to 172.16.28.$i"
 fi
done


