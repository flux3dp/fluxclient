Command Line Tools
=====================
FLUX SDK command line tools comes with :ref:`the installation of fluxclient <installation>`. You may use -h command to see command usage.

flux_discover
+++++++++++++++++++++

Discover FLUX Delta devices in local network.

Examples:

	``$ flux_discover``

	Find all devices and print in human-readable format continuously until recive `control + c`


	``$ flux_discover -t 60``

	Find all devices and print in human-readable format. flux_discover will exit after 60 secondes.


	``$ flux_discover -t 60 -f json``

	Find all devices and print in JSON format. flux_discover will exit after 60 secondes.


flux_upnp
+++++++++++++++++++++

Quick interactive console to manage device name, password, access control and network settings.

	``$ flux_upnp 192.168.1.2``

	Connect to device at 192.168.1.2


	``$ flux_upnp 423131420003e991a5c3faa4cc6a65ad``

	Find device which UUID is `423131420003e991a5c3faa4cc6a65ad` and connect to.


	``$ flux_upnp -a -p fluxdelta 423131420003e991a5c3faa4cc6a65ad``

	Find device which UUID is `423131420003e991a5c3faa4cc6a65ad`, use the password `fluxdelta` if password is required and add your authority key into device trust list and quick directly. If password is required and incorrect, flux_upnp will quit and return error.


flux_robot
+++++++++++++++++++++

Control flux device.

	``$ flux_robot 192.168.1.2``

	Connect to device at 192.168.1.2 and enter an interacvice shell. Like `flux_upnp`, you can give an IP address or UUID.


	``$ flux_robot 192.168.1.2 --shell ipython``

	Connect to device at 192.168.1.2 and embed an ipython shell. It is practical if you just want try to test or valide some API or your codes.


There are three default shell options for user:

	'simple' - This is a simple interactive shell for user.
	'curses' - Curses is an advance interactive shell but it has some bug.
	'ipython' - If you have IPython installed, this shell will invoke IPython shell after connected. You can use this shell to test your codes with device control.

Shell also support user custom shell. Here is an example:

1. Source code layout

	::

	myproject/
		__init__.py
		my_robot_shell.py

	::

2. my_robot_shell.py source code

	.. code-block:: python

		def my_shell(robot, device=None):
			# Note:
			# 'robot' argument is an instance of `fluxclient.robot.robot.Robot`
			# 'device' argument is an instance of `fluxclient.upnp.device.Device`. This argument maybe None if user give an IPAddress rather then an UUID.
			print("Here is connected robot object: %s" % robot)
			return 0

3. Run `flux_robot {UUID} --shell myproject.my_robot_shell.my_shell`


flux_camera
+++++++++++++++++++++

Grab photo from device camera

	``$ flux_camera 192.168.1.2``

	Connect to device at 192.168.1.2 and enter an interacvice shell. Like `flux_upnp`, you can give an IP address or UUID.

	``$ flux_camera 192.168.1.2 --fps 2 --path ~/live_ptoto``

	Limit 2 frames per seconds, save all recived photos into `~/live_ptoto`.

	``$ flux_camera 192.168.1.2 -f "%(target)s-%(time)s.jpg" --strftime %Y-%m-%d-%H-%M-%S-%f``

	Save photos with filename format `%(target)s-%(time)s.jpg`

	Format arguments

+--------------+------------------------------------------------+
| Argument     | Describe                                       |
+==============+================================================+
| %(target)s   | UUID or ip address pass to flux_camera.        |
+--------------+------------------------------------------------+
| %(ip)s       | Device IP.                                     |
+--------------+------------------------------------------------+
| %(uuid)s     | Device UUID. Accept only target is UUID.       |
+--------------+------------------------------------------------+
| %(model)s    | Device model name. Accept only target is UUID. |
+--------------+------------------------------------------------+
| %(name)s     | Device nickname. Accept only target is UUID.   |
+--------------+------------------------------------------------+
| %(serial)s   | Device serial. Accept only target is UUID.     |
+--------------+------------------------------------------------+
| %(index)s    | Stream photo index. Index start from 0.        |
+--------------+------------------------------------------------+
| %(time)s     | The time each photo recive at. Time format     |
|              | specific at `flux_camera` argument             |
|              | `--strftime`, read `python datetime doc`_ for  |
|              | more informations.                             |
+--------------+------------------------------------------------+

.. _python datetime doc: https://docs.python.org/3.3/library/datetime.html#strftime-strptime-behavior


flux_g2f
+++++++++++++++++++++

Convert gcode to fcode. If input file is not specific, it will try to read gcode from stdin. If output file is not specific, it will write fcode to stdout.

	``$ flux_g2f -i input.gcode output.fc``

	Convert gcode file to fcode.

	``$ flux_g2f -i input.gcode output.fc -t LASER``

	Convert gcode file to fcode. Set fcode file use an laster toolhead.


flux_f2g
+++++++++++++++++++++

Convert fcode to gcode. If input file is not specific, it will try to read fcode from stdin. If output file is not specific, it will write gcode to stdout.

	``$ flux_f2g -i input.gcode output.fc``

	Convert gcode file to fcode.
