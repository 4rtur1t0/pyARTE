#!/bin/sh
# Change INSTALLDIR to your needs. It currently points to your desktop
# the library's directory pyARTE should be placed at the same directory.
INSTALLDIR=~/Escritorio/pyARTE
UPDATELOG=~/Escritorio/pyARTE/update.log

while [ "$(hostname -I)" = "" ]; do
  echo -e "\e[1A\e[KNo network: $(date)"
  echo -e "\e[1A\e[KNo network: $(date)" >> $UPDATELOG
  sleep 1
done

echo "I have network"
echo "I have network" >> $UPDATELOG
echo "Internet is reachable"  >> $UPDATELOG

# none reachable
echo "Internet is up"
echo "Updating repos"
echo "Internet is up updating repos" >> $UPDATELOG

#sudo sudo -i -u usuario
# update repos
cd $INSTALLDIR
#git config --global --add safe.directory /home/usuario/Escritorio/pyARTE/
git stash >> $UPDATELOG 2>&1
git stash clear >> $UPDATELOG 2>&1
git pull >> $UPDATELOG 2>&1

#cd /home/usuario/Escritorio/ARTE/
##git config --global --add safe.directory /home/usuario/Escritorio/ARTE/
#git stash >> $UPDATELOG 2>&1
#git stash clear >> $UPDATELOG 2>&1
#git pull >> $UPDATELOG 2>&1

#chown -R usuario:usuario /home/usuario/Escritorio/pyARTE/
#chown -R usuario:usuario /home/usuario/Escritorio/ARTE/

echo "Process ended correctly" >> $UPDATELOG