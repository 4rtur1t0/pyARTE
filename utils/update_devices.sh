#
# USED TO update 
#
# Instrucciones:
# Ordenador profesor: >> sudo apt install sshpass
# Todos ordenadores instalar: >> sudo apt install openssh-server -y

# Final: ejecutar este script ./update_devices.sh

#!/bin/bash
for i in {2..100}
	do
	if ping -c1 172.16.28.$i 2>&1 2>/dev/null; then
		echo "Found up IP: 172.16.28.$i"
		sshpass -pusuario scp rc.local usuario@172.16.28.$i:/home/usuario/rc.local
		sshpass -pusuario ssh usuario@172.16.28.$i "echo 'usuario' | sudo -S -k mv /home/usuario/rc.local /etc/rc.local"
		echo "SUCCESSSSSSSSSSSSSSSSSSSS ************************************"
	else
		echo "Could not connect to 172.16.28.$i"
	fi
done

