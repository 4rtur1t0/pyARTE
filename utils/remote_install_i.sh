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

COPPELIA_PATH=~/Applications/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/
ARTE_PATH=~/Escritorio/ARTE/
pyARTE_PATH=~/Escritorio/pyARTE/

DESKTOP_COPPELIA_FILE=~/Escritorio/coppeliasim.desktop
DESKTOP_MATLAB_FILE=~/Escritorio/Matlab.desktop
DESKTOP_PYCHARM_FILE=~/Escritorio/pycharm.desktop

address=$1

echo "trying to install address $address"

#!/bin/bash
if ping -c1 172.16.28.$address 2>&1 2>/dev/null; then
  echo "Found up IP: 172.16.28.$address"
#  echo "Transferring Coppelia Sim"
#  sshpass -p usuario scp -r $COPPELIA_PATH usuario@172.16.28.$i:~/Applications

  echo "Cloning ARTE"
  sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
  cd $ARTE_PATH && \
  git fetch --all && \
  git reset --hard origin/master
  "

  echo "Cloning pyARTE"
  sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
  cd $pyARTE_PATH && \
  git fetch --all && \
  git reset --hard origin/master
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


