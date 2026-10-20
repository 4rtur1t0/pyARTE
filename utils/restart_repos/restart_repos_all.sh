#!/bin/bash
!/bin/bash
OWN_IP=$(hostname -I | tr ' ' '\n' | grep '^172\.16\.28\.' | cut -d'.' -f4 | head -n 1)
echo "IP pròpia detectada: 172.16.28.$OWN_IP"


MAX_JOBS=50  # Nombre d'equips que s'instal·laran en paral·lel

echo "Iniciant repos de forma paral·lela ($MAX_JOBS equips simultanis)..."

seq 1 200 | grep -v "^${OWN_IP}$" | xargs -P $MAX_JOBS -I {} ./restart_repo_i.sh {}

echo "=== Procés global finalitzat ==="


