# README #

**ARTEpy** is a Python toolbox focussed on robotic manipulators.

The library uses Coppelia Sim

The main features of **ARTEpy** are:

* Simulate some industrials robot within the Coppelia Sim environment.
* Path planning and obstacle avoidance.
* Practical exercises are provided.

Created by [Arturo Gil](http://arvc.umh.es/personal/arturo/index.php?lang=en&vista=normal&dest=inicio&idp=arturo&type=per&ficha=on): arturo.gil@umh.es. [Miguel Hernández University, Spain.](http://www.umh.es)

ARTEpy is distributed under LGPL license.


**INSTALL**

Clone this repository:

```
>> git clone https://github.com/4rtur1t0/pyARTE.git
```

Download and extract CoppeliaSim on your PC. Download Coppelia from https://www.coppeliarobotics.com/downloads.

Next, create a directory named coppelia_API:
```
>> mkdir coppelia_API
```
Look for the following directory in CoppeliaSim install directory: 
CoppeliaSim/programming/remoteApiBindings/python/python
copy its contents to the directory named coppelia_API. 
Locate the following file and copy it to the directory named coppelia_API
CoppeliaSim/programming/remoteApiBindings/lib/Ubuntu20\_04/remoteApi.so

If using Windows or Mac, you should look for your corresponding system library interface.

Open the project in Pycharm (or any other python editor). Use the option ''add root content'' in 
File-Settings-Project Structure-- Add Content Root. Add the directory coppelia_API.

**PRACTICALS**

Practical exercises are provided under the practicals directory.
You can directly execute the practicals/solutions scripts that are provided on the develop branch.

**YOUTUBE CHANNEL**
http://www.youtube.com/playlist?list=PLClKgnzRFYe72qDYmj5CRpR9ICNnQehup

**PRACTICAL EXERCISES WITH THE LIBRARY**
http://arvc.umh.es/arte/index_en.html#practicals
