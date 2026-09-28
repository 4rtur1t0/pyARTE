#!/bin/bash

# USED TO update every computer, given its IP
# Copy Coppelia Sim Directory
# Copy ARTE directory
# Copy pyARTE directory

ARTE_PATH=~/Escritorio/ARTE/
pyARTE_PATH=~/Escritorio/pyARTE/
INTRO2AI_PATH=~/Escritorio/Intro2AI/

address=$1
SERVIDOR="172.16.28.$address"

echo "trying to UPDATE address SERVIDOR"
echo "INTENTANDO CONECTAR con IP" $SERVIDOR

# POR SIMPLICIDAD, SALIMOS SI EL SERVIDOR NO RESPONDE
nc -z -w 5 "$SERVIDOR" 22 || exit 1

echo "=== Conectando a $SERVIDOR mediante contraseña I ACTUALITZANT REPOS ==="


echo "UPDATING ARTE"
sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
cd $ARTE_PATH && \
git fetch --all && \
git reset --hard origin/master
"

echo "UPDATING pyARTE"
sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
cd $pyARTE_PATH && \
git fetch --all && \
git reset --hard origin/main
"

echo "UPDATING INTRO2AI"
sshpass -p usuario ssh -o StrictHostKeyChecking=no usuario@172.16.28.$address "
cd $INTRO2AI_PATH && \
git fetch --all && \
git reset --hard origin/main
"


#  echo "Copy file"
#  FILE_TTT=~/Escritorio/pyARTE/practicals/projects/year_2526_welding/irb140_project_2526_welding.ttt
#  sshpass -p usuario scp -r $FILE_TTT usuario@172.16.28.$i:$FILE_TTT
echo "ENDED SCRIPT"



