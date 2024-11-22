#
# USED TO update all computers
#
# Instrucciones:
# Ordenador profesor: >> sudo apt install sshpass
# Todos ordenadores instalar: >> sudo apt install openssh-server -y
# Final: ejecutar este script ./update_computers.sh

UPDATE_PATH=~/Escritorio/pyARTE/utils
UPDATEFILE=$UPDATE_PATH/update.sh
UPDATELOG=$UPDATE_PATH/update.log
croncmd="$UPDATEFILE > $UPDATELOG"
cronjob="@reboot $croncmd"

#To add it to the crontab, with no duplication:
#( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -"
#To remove it from the crontab whatever its current schedule:
#( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -
# add to a single command
# this command executes crontab and add the desired line
# ARTURO: commandssh="( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -"
commandssh="( crontab -l 1>/dev/null || echo "" | crontab -; crontab -l | grep -v -F \"$cronjob\" ; echo \"$cronjob\" ) | crontab -"


echo "Command to execute is: $commandssh"

#!/bin/bash
for i in {2..100}
 do
 if ping -c1 172.16.28.$i 2>&1 2>/dev/null; then
  echo "Found up IP: 172.16.28.$i"
  sshpass -pusuario ssh -oStrictHostKeyChecking=no usuario@172.16.28.$i "mkdir -p $UPDATE_PATH"
  #sshpass -pusuario scp update.sh usuario@172.16.28.$i:$UPDATEFILE
  # sshpass -pusuario ssh usuario@172.16.28.$i "echo 'usuario' | sudo -S -k mv /home/usuario/rc.local /etc/rc.local"
  # No need for pswd: sshpass -pusuario ssh usuario@172.16.28.$i "echo 'usuario' | $commandssh"
  sshpass -pusuario ssh -oStrictHostKeyChecking=no usuario@172.16.28.$i "$commandssh"
  echo "[SUCCESS] Updated crontab command to update repository in IP: 172.16.28.$i"
#  sshpass -pusuario ssh -oStrictHostKeyChecking=no usuario@172.16.28.$i "echo 'usuario' | sudo -S -k halt"
  sshpass -pusuario ssh -oStrictHostKeyChecking=no usuario@172.16.28.$i "echo 'usuario' | sudo -S -k shutdown -P 1"
 else
  echo "[ERROR] Could not connect to 172.16.28.$i"
 fi
done


