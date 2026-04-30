#!/bin/sh
# Change INSTALLDIR to your needs. It currently points to your desktop
# the library's directory pyARTE should be placed at the same directory.
INSTALLDIR=~/Escritorio/pyARTE
REQUIREMENTS=~/Escritorio/pyARTE/requirements.txt
PIP=~/Applications/venv/bin/pip

while [ "$(hostname -I)" = "" ]; do
  echo -e "\e[1A\e[KNo network: $(date)"
  sleep 1
done

echo "Network available. Internet is reachable. Updating repositories:"

# update repos
cd $INSTALLDIR
git stash
git stash clear
git pull

echo "Changing permissions to user"
chown -R usuario:usuario $INSTALLDIR

# now install packages WITh pip
$PIP install -r $REQUIREMENTS


#################### 
## CHECKS COPPELIA VERSION INSTALLED AND THE ONE CONFIGURED IN PYCHARM WORKSPACE TO MATCH THEM
version_coppelia=$(grep -oP 'Applications/(Coppelia[^/]+)/' ~/Escritorio/coppeliasim.desktop)
version_pycharm=$(grep -oP 'Applications/(Coppelia[^/]+)/' ~/Escritorio/pyARTE/.idea/workspace.xml)

echo "Coppelia version detected in the system is: $version_coppelia"
echo "Coppelia version configured in pycharm: $version_pycharm"


if [ "$num1" -eq "$num2" ]; then
  echo "Both version detected are the same, no need to update."
else
  echo "Coppelia version differs from the one configured in workspace and the one installed in the system."
  echo "Updating Workspace XML configuration:"
	sed -i "s|$version_pycharm|$version_coppelia|g" ~/Escritorio/pyARTE/.idea/workspace.xml
fi
####################

echo "Changing permissions to user"
chown -R usuario:usuario $INSTALLDIR

echo "Process ended correctly on
date
