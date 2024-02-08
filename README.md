# README

**pyARTE** is a Python toolbox focussed on robotic manipulators.

The library uses Coppelia Sim as the main way to produce a representation of the robots and methods.

The main features of **pyARTE** are:

* Simulate some industrials robot within the Coppelia Sim environment.
* Path planning and obstacle avoidance.
* Practical exercises are provided.

Created by [Arturo Gil](http://arvc.umh.es/personal/arturo/index.php?lang=en&vista=normal&dest=inicio&idp=arturo&type=per&ficha=on): arturo.gil@umh.es. [Miguel Hernández University, Spain.](http://www.umh.es)

pyARTE is distributed under LGPL license.

![Screenshot](screenshot.png)


# INSTALL

## DOWNLOAD THE CODE
Clone this repository:

```
>> git clone https://github.com/4rtur1t0/pyARTE.git
```

## DOWNLOAD AND INSTALL COPPELIA SIM
 
Download and extract CoppeliaSim on your PC. Download Coppelia from https://www.coppeliarobotics.com/downloads.
Download the EDU version of Coppelia, if pyARTE and Coppelia Sim are used for educational purposes.
pyARTE has been tested under Ubuntu 20.04 and 22.04. It has not been tested under Mac/Windows.

Extract CoppeliaSim to:
```
/home/user/Simulations
```


## INSTALL SYSTEM PACKAGES
Install some needed packages  (python3-tk is needed in some distributions to plot with matplotlib)
```
>> sudo apt install virtualenv
>> sudo apt install python3-dev
>> sudo apt install git
>> sudo  apt-get install python3-tk
```

## CREATE A VIRTUAL ENVIRONMENT
We will be creating a virtual environment at your user's home/Simulations directory: 
```
>> cd
>> mkdir Simulations
>> cd Simulations
>> virtualenv venv
```

Next, install some needed python packages. We only require matplotlib, numpy and pynput:
```
>> cd /home/user/Simulations/venv/bin
>> ./pip3 install numpy matplotlib pynput pyzmq cbor opencv-contrib-python
```

## CONFIGURE YOUR IDE

Open the project in Pycharm (or any other python editor). Use the option ''add root content'' in 
File-Settings-Project Structure-- Add Content Root. Under the root content (the files that will be included in the python
execution), you should include:
- Add the directory with the zmQ communications library that is included with Coppelia Sim. Generally, the remote zmq Api can be found in:
  /home/user/Simulations/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src
- Check that the directory pyARTE is also included.

Alternatively, the directory path with the Zmq libraries can also be added to the PYTHONPATH variable:
$ export PYTHONPATH=/home/usuario/Simulations/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src


## TEST
Open Coppelia Sim.
Open the irb140.ttt scene. This scene is found in pyARTE/scenes/irb140.ttt.
Open and execute the pyARTE/practicals/irb140_first_script.py
Open and execute the pyARTE/practicals/applications/irb140_palletizing_color.py

The python scripts should connect to Coppelia and start the simulation.

##SCENES
The directory pyARTE/scenes includes a set of scenes that should be opened in Coppelia Sim in order to execute the demos or tutorials in this library.

## INSTALL SCRIPT

Of course, Coppelia Sim should be downloaded from their webpage and extracted to a directory.

IMPORTANT: if using pycharm or Visual Studio, you must include the communications library between Coppelia and python.
In pycharm: File--> Settings --> Project --> Project Structure --> Add root content --> Add the directory with the zmQ communications library that is included with Coppelia Sim. Generally, the remote zmq Api can be found in:
  /home/user/Simulations/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src

Alternatively, you can also add the libraries directly to the python path
$ export PYTHONPATH=/home/usuario/Simulations/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src

Just copy-paste to your terminal the following lines to get everything up and running in seconds.

```
# Change INSTALLDIR to your needs. It currently points to your desktop
# the library's directory pyARTE should be placed at the same directory.
INSTALLDIR=~/Simulations
mkdir $INSTALLDIR
cd $INSTALLDIR

# this installs the virtualenv package and python and git
sudo apt install virtualenv
sudo apt install python3-dev
sudo apt install git
sudo  apt-get install python3-tk


# cloning repository SKIP IT IF YOU ALREADY HAD
git clone https://github.com/4rtur1t0/pyARTE

# Create a virtual environment and install dependencies 
cd $INSTALLDIR
virtualenv venv
cd venv/bin
./pip install -r ../../pyARTE/requirements.txt
```
