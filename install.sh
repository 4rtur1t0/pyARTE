# Change INSTALLDIR to your needs. It currently points to your desktop
# the library's directory pyARTE should be placed at the same directory.
INSTALLDIR=~/Simulations
mkdir $INSTALLDIR

echo 'INSTALLING pyARTE in'
echo $INSTALLDIR
read -p "Do you want to proceed? (y/n) " choice
case $choice in
[yY]* ) echo "Ok, installing" ;;
[nN]* ) exit ;;
*) exit ;;
esac

echo 'INSTALLING virtualenv, python3-dev and git'
read -p "Do you want to proceed? (y/n) " choice
case $choice in
[yY]* ) echo "Ok, installing" ;;
[nN]* ) exit ;;
*) exit ;;
esac

# this installs the virtualenv package and python and git
sudo apt install virtualenv
sudo apt install python3-dev
sudo apt install git

cd ~
cd $INSTALLDIR

echo 'CLONING repository pyARTE from github. Say no if you already cloned it'
read -p "Do you want to proceed? (y/n) " choice
case $choice in
[yY]* ) git clone https://github.com/4rtur1t0/pyARTE ;;
[nN]* ) exit ;;
*) exit ;;
esac

echo 'NOW creating a virtual environment and installing requirements with pip in it.'
echo $INSTALLDIR
read -p "Do you want to proceed? (y/n) " choice
case $choice in
[yY]* ) git clone https://github.com/4rtur1t0/pyARTE ;;
[nN]* ) exit ;;
*) exit ;;
esac

# move to the home directory
cd ~
cd $INSTALLDIR
echo 'CREATING VIRTUAL ENVIRONMENT IN'
echo $INSTALLDIR
virtualenv venv
cd venv/bin
pwd
echo 'INSTALLING DEPENDENCIES WITH PIP'
./pip install -r ../../pyARTE/requirements.txt


