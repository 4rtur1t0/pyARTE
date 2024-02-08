# Change INSTALLDIR to your needs. It currently points to your desktop
# the library's directory pyARTE should be placed at the same directory.
INSTALLDIR=`xdg-user-dir DESKTOP`

# this installs the virtualenv package
sudo apt install virtualenv
sudo apt install python3-dev

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
cd $INSTALLDIR

