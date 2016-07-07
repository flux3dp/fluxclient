## Fluxclient

Fluxclient is all you need to play with your Delta making fun project. It provide APIs to control your machine in every possible ways. It's written mainly in [Python](python.org) with some C++ extensions powered by [Cython](http://cython.org/). 

Official site: http://flux3dp.com/  
Official forum: http://forum.flux3dp.com/  
Official documentation: http://some.cool.name.flux3dp.com/  

## Features

* API for controlling your Delta machine.
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

## installation
* install [`python 3.4+`]((python.org)) and [`pip`](https://pypi.python.org/pypi/pip)  
* Pillow: `pip install Pillow`

* Numpy: `pip install numpy`

* Scipy: `pip install scipy`

* pcl(optional): `brew install pcl --without-apps --without-qt`

* fluxclient:
```
git clone https://github.com/flux3dp/fluxclient  
python setup.py install
```

## Quick Start
### commands
```
* flux_discover
> find FLUX 3D Printer in the LAN.

* flux_auth
> Try to grant access permission to printer

* flux_passwd
> Change password for printer. It password is exist, old password is required

* flux_config_network
> Set printer network

* flux_g2f -i input.gcode -o output.fc
> convert gcode to fcode
```

### sdk: see [document website](google.com) for APIs usage
```
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

* see our [official documentation](google.com)

* To generate documents by yourself

`sudo python setup.py build_sphinx`  
`open ./build/sphinx/html/index.html`

## Community
You can ask question and discuss with our engineers on our [official forum](http://forum.flux3dp.com/).
Issues are being tracked here on GitHub  

## License

`fluxclient`'s code in this repo uses the `AGPLv3` license, see our [`LICENSE`](https://github.com/flux3dp/fluxclient/blob/master/LICENSE) file. 
