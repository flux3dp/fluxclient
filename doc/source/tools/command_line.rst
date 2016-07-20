Command Line Tools
=====================
FLUX SDK command line tools comes with :ref:`the installation of fluxclient <installation>`. You may use -h command to see command usage.

Network Tools
+++++++++++++++++++++

:: 
	
	$ flux_discover
	Description: Finds FLUX Delta devices in local network.

::

	$ flux_auth
	Description: Grants access permission to the device

::

	$ flux_passwd
	Description: Changes device password. If password exists, old password is required

::

	$ flux_config_network
	Description: Sets network configuration

GCode Conversion Tools
+++++++++++++++++++++

::

	$ flux_g2f
	Description: Converts gcode to fcode

**Usage Example**

::

	$ flux_g2f -i caligraphic.gcode -o ~/Desktop/caligraphic_example.fc --type N/A

**Options**
::

	-i Input file path
	-o Output file path
	--type {EXTRUDER,LASER,N/A}
		Set toolhead type for this gcode, if the type does not match the toolhead, the machine will pause the task automatically. Default value is EXTRUDER. EXTRUDER = Printing toolhead, LASER = Engraving toolhead, N/A = No specific toolhead ( or drawing toolhead ).

	--cor {A,H,N}
		Auto calibration settings. A = ALL, H = Height Only, N = No.

	--hel HEAD_ERROR_LEVEL
		Toolhead error detection level

	--fmd {Y,N}          
		Filament detection (on/off). Only for extruder type.