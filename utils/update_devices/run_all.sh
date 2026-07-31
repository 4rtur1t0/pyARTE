#!/bin/bash
own_ip=51
for i in {1..100}; do
    if [ "$i" -eq "$own_ip" ]; then
      echo "OWN IP"
      continue
    fi
    echo "Valor de i: $i"
    ./remote_install_i.sh $i
done


