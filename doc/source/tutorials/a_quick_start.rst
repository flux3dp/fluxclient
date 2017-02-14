A Quick Start
****************

Welcome! This guide will show you how to control FLUX Delta's movement in FLUX Delta's SDK mode.

.. note:: 
	Required firmware version for FLUX Delta is **v1.3+**.
	You can download it from `here <http://www.mediafire.com/download/g0j49029kc81tb3/fluxfirmware-1.3a1.fxfw>`_.

Finding IP Address
++++++++++++++++
You can get the IP address of your FLUX Delta by running utility program *flux_discover* in terminal.

.. code-block:: bash

	$ flux_discover
	## Returns ##

	                            UUID Serial     PWD       ## Name
	                         IP Addr Version    Model     
	===============================================================================
	46314b30002f6c86d2b02c73dead910b F1K0690047 YES       ## Simon's Delta
	                   192.168.18.33 1.1.7      delta-1   

The IP of FLUX Delta is **192.168.18.33**. 
To know how to fetch device IP programmatically, kindly check `the guide of discovering devices </tutorial/device/discovering_devices.html>`_.

Establishing a connection in SDK mode
++++++++++
To start controlling the motion of the machine, you'll have to establish a connection with FLUX Delta, the class :class:`fluxclient.sdk.delta.Delta` provides a simple way to connect machine, and switch the connection into SDK mode ( free motion ). Controlling task status is described in here.

Open up your favorite text editor, paste and modify following example code:

.. code-block:: python
	:name: connect_flux.py

	from fluxclient.sdk.delta import Delta

	# Connects to machine
	my_delta = Delta.connect_delta(ip='192.168.18.33', password='flux', kick=True, blocking=False)

	# Moves to origin
	my_delta.home()

	# Moves the toolhead to position:(0, 0, 80)
	my_delta.move(0, 0, 80)

	# Moves to origin
	my_delta.home()

	# Disconnects with the machine
	my_delta.close()

Save it into ``connect_flux.py``, and run ``python3 connect_flux.py``, then you will see the machine moving according to your commands.

.. note:: The parameter ``blocking`` in :func:`fluxclient.sdk.delta.Delta.connect_delta` means that local computer send new commands only if the machine finished prior commands.


More commands
+++++++++
Kindly check documentation of :class:`fluxclient.sdk.delta.Delta`.