## FLUXClient

FLUXClient is all you needed to do cool projects and develop new features with your FLUX Delta. It provides APIs to control your machine in every possible way. It's written mainly in [Python](python.org) with some C++ extensions powered by [Cython](http://cython.org/). 

Official site: http://flux3dp.com/  
Official forum: http://forum.flux3dp.com/  
Official documentation: http://dev.flux3dp.com/  

## Features

* API for controlling your FLUX Delta.
* Built-in commands:
  * gcode <-> fcode converter
  * delta discover
  * delta's authorization settings

* Flux Studio's backend service:
  * printing with [CuraEngine](https://github.com/daid/Cura) or [Slic3r](https://github.com/alexrj/Slic3r)
  * svg file parser
  * laser fcode generate from bitmap or svg file
  * pen drawing fcode generate from svg file
  * scanning

## Installation

### Mac OS X
* Install [`python 3.4+`](https://python.org) and [`pip`](https://pypi.python.org/pypi/pip)  
* Pillow: `pip install Pillow`
* Numpy: `pip install numpy`
* Scipy: `pip install scipy`
* Setuputils: `pip install setuputils`
* pcl(optional): `brew install pcl --without-apps --without-qt`

* Compile and install FLUXClient:
```
git clone https://github.com/flux3dp/fluxclient  
cd fluxclient
python3 setup.py install
```

### Windows
* Install Visual Studio 2015 Community ( Select only VC++ and Python Tools )
* Install Anaconda ( Special edition of Python 3.5 ) from [here](https://www.continuum.io/downloads)
* Install Precompiled PCL 1.7.2 for Win64 for MSVC 2015 from
[here](http://unanancyowen.com/?p=712)
* Install [Github Desktop](https://desktop.github.com/) ( or git for windows )
* Pillow: `pip install pillow`
* Numpy: `pip install numpy`
* Pycrypto: `pip install pycrypto`
* Cython: `pip install Cython`

* Compile and install FLUXClient:
```
git clone https://github.com/flux3dp/fluxclient  
cd fluxclient
python3 setup.py install
```

## Quick Start
### Utility programs
```
$ flux_discover
Description: Discover FLUX 3D Printer in the LAN.

$ flux_upnp "device uuid"|"ip address"
Description: Fast manage device name, password, access control and network settings.

$ flux_robot "device uuid"|"ip address"
Description: Control flux device.

$ flux_camera "device uuid"|"ip address"
Description: Grab photo from device camera

$ flux_g2f -i input.gcode output.fc
Description: convert gcode to fcode

$ flux_f2g -i input.fc output.gcode
Description: convert fcode to gcode
```

### SDK: Check [this guide](http://dev.flux3dp.com/tutorials/a_quick_start.html) for a quick start
```
from fluxclient.sdk.delta import Delta 

# connect to machine
my_delta = Delta.connect_delta(ip='192.168.18.135', password='flux', kick=True)

# home
my_delta.home()

# move the tool head to position:(0, 0, 80)
my_delta.move(0, 0, 80)

# turn on the left side laser(red laser)
my_delta.turn_laser('L', True)

# disconnect with delta
my_delta.close()
```

## Documents

* See [official documentation](http://dev.flux3dp.com/API/Documentation.html)

* To generate documents by yourself:

`sudo python3 setup.py build_sphinx`  
`open ./build/sphinx/html/index.html`

## Community
You can ask questions and discuss with our developers on [FLUX Official Forum](http://forum.flux3dp.com/).
Issues are being tracked here on GitHub  

## License

`fluxclient`'s code in this repo uses the `AGPLv3` license, see our [`LICENSE`](https://github.com/flux3dp/fluxclient/blob/master/LICENSE) file. 
