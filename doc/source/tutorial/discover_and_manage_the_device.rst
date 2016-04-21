Discover and manage the device
********************************

.. automodule:: fluxclient.upnp

Discover devices
==================

The `fluxclient.upnp.Discover` class provide interface to discover and collect informations continuously.

.. automodule:: fluxclient.upnp.discover

.. autoclass:: fluxclient.upnp.discover.UpnpDiscover
   :members:

Device metadata
=================

Metadata is a dict store vary device information from discover result.

Metadata contain at least follow key-values:

* uuid (uuid.UUID) - Device UUID
* name (str) - Device name
* serial (str) - Device serial
* model_id (str) - Model ID
* version (str) - Device firmware version
* ipaddr (str) - Device IP address
* endpoint ((str, int)) - Endpoint device response from

Metada has other usable optional values:

* st_id (int) - Device current status ID.
* st_ts (int) - Timestemp (Device status last update).
* st_prog (float) - Prograss only use when device is running task.
* head_module (str) - Toolhead currently installed, **only appear** when device is running task.
* error_label (str) - This value available when device gets error while running task.

.. list-table:: Device status ID
    :widths: 20 40
    :header-rows: 1

    * - st_id
      - Describe
    * - 0
      - Idle
    * - 1
      - Init a task
    * - 4
      - Starting a task
    * - 64
      - Running a task
    * - 36, 48
      - Paused a task
    * - 38, 50
      - Pausing a task
    * - 6, 18
      - Resuming a task
    * - 66
      - Completing a task
    * - 64
      - A task is completed
    * - 130
      - Aborting a task
    * - 128
      - A task is aborted
    * - -1
      - Occupied for maintain 
    * - -2
      - Occupied for scan 

Authorize and manage the device
=================================

FLUX device use password and RSA key for access authorize. At first time user connect to device, a RSA key and password is required. After password authorized, Only RSA key is required.

.. autoclass:: fluxclient.upnp.task.UpnpTask

When create UpnpTask instance, the argument **uuid** is required. If param **device_metadata** not given, UpnpTask will use lookup_callback and lookup_timeout to create a Discover instance and try to get metadata from network.

.. note:: Note: Assign **device_metadata** can skip discover process in UpnpTask constructor.

Authorize with password
+++++++++++++++++++++++++

To authorize with password, simply add a key value pair to param **backend_options**::

  backend_options = {"password": "YOUR_ACCESS_PASSWORD"}

**password** key also accept a callable object as value like::

  # instance is the UpnpTask instance who call the lambda
  backend_options = {"password": lambda instance: getpass("Please input device password: ")}

Manage device name, network and security
++++++++++++++++++++++++++++++++++++++++++

.. automethod:: fluxclient.upnp.task.UpnpTask.rename
.. automethod:: fluxclient.upnp.task.UpnpTask.modify_password
.. automethod:: fluxclient.upnp.task.UpnpTask.modify_network
.. automethod:: fluxclient.upnp.task.UpnpTask.get_wifi_list

.. sectionauthor:: Cerberus Yagami <cerberus@flux3dp.com>
