.. _installation:

Installation
==============
`FLUXClient <https://github.com/flux3dp/fluxclient>`_ is the only library you needed to do cool projects and develop new features with your `FLUX Delta <http://flux3dp.com>`_. It provides APIs to control your machine in every possible way. 

Here's how you install it:

.. TODO: Install cython, setup.py install

OS X or Linux
------------------------------
* Install `Python 3.4+ <http://python.org>`_ and `pip <https://pypi.python.org/pypi/pip>`_
* Pillow: ``pip install Pillow``
* Numpy: ``pip install numpy``
* Scipy: ``pip install scipy``
* Setuputils: ``pip install setuputils``
* pcl(optional): ``brew install pcl --without-apps --without-qt``

* Compile and install FLUXClient

.. code-block:: bash

    $ git clone https://github.com/flux3dp/fluxclient  
    $ cd fluxclient
    $ python3 setup.py install

Windows
------------------------------
* Install Visual Studio 2015 Community ( Select only VC++ and Python Tools )

* Install Anaconda ( Special edition of Python 3.5 ) from `here <https://www.continuum.io/downloads>`_

* Install Prebuilt PCL 1.7.2 for MSVC 2015 from `here <http://unanancyowen.com/?p=712>`_

* Install `Github Desktop <https://desktop.github.com/>`_ ( or git for windows )
* Pillow: ``pip install pillow``

* Numpy: ``pip install numpy``

* Pycrypto: ``pip install pycrypto``

* Cython: ``pip install Cython``

* Compile and install FLUXClient

.. code-block:: bash

    $ git clone https://github.com/flux3dp/fluxclient  
    $ cd fluxclient
    $ python3 setup.py install

.. Linking Slicing Engine
.. ------------------------------

.. Slicing engine are **only required** if you are going to use FLUXClient slicing API.

.. There are two open-source slicing engines can be utilized by FLUXClient.

.. * `Slic3r (Compatible with latest version) <http://slic3r.org/>`_

.. * `Cura (Compatible with v15.04.5) <https://ultimaker.com/en/products/cura-software/list>`_