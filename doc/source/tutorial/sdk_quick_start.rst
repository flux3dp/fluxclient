SDK Mode Quick Start
****************

This chapter will show you how to control FLUX Delta's movement programmatically in FLUX Delta's SDK mode.


Finding device's IP
++++++++++++++++
You can get your device's IP address by running utility program *flux_discover* in terminal

.. code-block:: bash

	$> flux_discover
	## Returns ##

	                            UUID Serial     PWD       ## Name
	                         IP Addr Version    Model     
	===============================================================================
	46314b30002f6c86d2b02c73dead910b F1K0690047 YES       ## Simon's Delta
	                   192.168.18.33 1.1.7      delta-1   

So the IP of FLUX Delta is **192.168.18.33**.

Connecting machine
++++++++++

Open up your favorite text editor, paste and modify following example code:

.. code-block:: python
	:name: connect_flux.py

	from fluxclient.sdk.delta import Delta

	# connect to machine
	my_delta = Delta.connect_delta(ip='192.168.18.33', password='flux', kick=True)

	# home
	my_delta.home()

	# move the tool head to position:(0, 0, 80)
	my_delta.move(0, 0, 80)

	# home
	my_delta.home()

	# disconnect with delta
	my_delta.close()

Save it into ``connect_flux.py``, and run ``python3 connect_flux.py``, then you will see the machine moving according to your commands.


More commands
+++++++++
Check this