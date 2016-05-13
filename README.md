# fluxclient

fluxclient contains all toolsets to control FLUX 3D Printer

## Commands
* flux_discover
> find FLUX 3D Printer in the LAN.

* flux_auth
> Try to grant access permission to printer

* flux_passwd
> Change password for printer. It password is exist, old password is required

* flux_config_network
> Set printer network


### Install require package

#### On mac with homebrew

* pcl:
`brew install pcl --without-apps --without-qt`

* Pillow
`pip install Pillow`

* Numpy
`pip install numpy`

* Scipy
`pip install scipy`


## Document
* sphinx

`sudo python setup.py build_sphinx`
`open ./build/sphinx/html/index.html`
