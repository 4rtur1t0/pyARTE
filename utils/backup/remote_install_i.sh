#
# USED TO update every computer, given its IP
# Copy Coppelia Sim Directory
# Copy ARTE directory
# Copy pyARTE directory
#
# Instrucciones:
# Ordenador profesor: >> sudo apt install sshpass
# Todos ordenadores instalar: >> sudo apt install openssh-server -y
# Final: ejecutar este script ./install_computers.sh

COPPELIA_PATH=~/Aplicaciones/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/
PYCHARM_PATH=~/Aplicaciones/pycharm-2026.2.0.1
ARTE_PATH=~/Escritorio/
pyARTE_PATH=~/Escritorio/

DESKTOP_COPPELIA_FILE=~/Escritorio/coppeliasim.desktop
DESKTOP_MATLAB_FILE=~/Escritorio/Matlab.desktop
DESKTOP_PYCHARM_FILE=~/Escritorio/pycharm.desktop

address=$1

echo "trying to install address $address"

#!/bin/bash
if ping -c1 172.16.28.$address 2>&1 2>/dev/null; then
  echo "Found up IP: 172.16.28.$address"

  echo "Transferring Coppelia Sim"
  sshpass -p usuario scp -r $COPPELIA_PATH usuario@172.16.28.$address:~/Applications
  echo "Transferring pyCharm"
  sshpass -p usuario scp -r $PYCHARM_PATH usuario@172.16.28.$address:~/Applications

  echo "Install libraries"
  sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
  sudo apt update && \
  sudo apt install git python3 python3-virtualenv
  "
# create virtual environment at Escritorio

# install pip libraries at the virtual environment

  echo "Cloning ARTE"
  sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
  cd $ARTE_PATH && \
  git clone https://github.com/4rtur1t0/ARTE
  "

 echo "Cloning pyARTE"
  sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
  cd $PYARTE_PATH && \
  git clone https://github.com/4rtur1t0/pyARTE
  "
#  echo "Copy file"
#  FILE_TTT=~/Escritorio/pyARTE/practicals/projects/year_2526_welding/irb140_project_2526_welding.ttt
#  sshpass -p usuario scp -r $FILE_TTT usuario@172.16.28.$i:$FILE_TTT

#  echo "Transferring Desktop files"
#  sshpass -p usuario scp $DESKTOP_COPPELIA_FILE usuario@172.16.28.$i:~/Escritorio
#  sshpass -p usuario scp $DESKTOP_MATLAB_FILE usuario@172.16.28.$i:~/Escritorio
#  sshpass -p usuario scp $DESKTOP_PYCHARM_FILE usuario@172.16.28.$i:~/Escritorio
  echo "ENDED Transfer"
 else
  echo "[ERROR] Could not connect to 172.16.28.$address"
 fi
done


