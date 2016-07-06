## Fluxclient

Official site: http://flux3dp.com/  
Official forum: http://forum.flux3dp.com/  
Official documentation: http://some.cool.name.flux3dp.com/  

Fluxclient is all you need to play with your Delta making fun project. It provide APIs to control your machine in every possible ways. It's written mainly in [Python](python.org) with some C++ extensions powered by [Cython](http://cython.org/). 

## Features

* API for controlling your Delta machine.
* Built-in commands:
  * gcode <-> fcode converter
  * delta discover
  * delta's authorization settings

* Flux Studio's backend service:
  * printing with Cura or Slic3r
  * svg file parser
  * laser fcode generate from bitmap or svg file
  * pen drawing fcode generate from svg file
  * scanning

## installation
* Pillow
`pip install Pillow`

* Numpy
`pip install numpy`

* Scipy
`pip install scipy`

* pcl:
`brew install pcl --without-apps --without-qt`

* fluxclient
`
git clone https://github.com/flux3dp/fluxclient  
python setup.py install
`

## Quick Start
```
* flux_discover
> find FLUX 3D Printer in the LAN.

* flux_auth
> Try to grant access permission to printer

* flux_passwd
> Change password for printer. It password is exist, old password is required

* flux_config_network
> Set printer network
```

## Documents

* see our [official documentation](google.com)

* To generate documents by yourself

`sudo python setup.py build_sphinx`
`open ./build/sphinx/html/index.html`

## Community

http://forum.flux3dp.com/  

## License

`fluxclient`'s code in this repo uses the `AGPLv3` license, see our `LICENSE` file. 
