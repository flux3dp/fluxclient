Authorization and Configuration
=================================

FLUX Device use password and RSA key to authorizing access. At the first time a user connect to device, a RSA key and password is required. After the password is authorized, 
you can insert your RSA key into device trusted key list.

.. note:: Note: You must add your RSA Key to device trusted list because robot and camera service does not accept password authorize.
 

Authorize with password
+++++++++++++++++++++++++

When create a DeviceManager instance, call :func:`fluxclient.device.manager.DeviceManager.authorized` to ensure your connection grant permission to operation. If `authorized` return False, you have to call :func:`fluxclient.device.manager.DeviceManager.authorize_with_password` to complete authorize.::

    manager = DeviceManager(uuid)
    if manager.authorized:
        pass  # authorized

    else:
        password = getpass("Password: ")
        try:
            manager.authorize_with_password(password)
            print("Authorized")
        except ManagerError as e:
            print("Authorize failed: %s" % e)
            raise


Manage device name, network and security
++++++++++++++++++++++++++++++++++++++++++

.. autoclass:: fluxclient.device.manager.DeviceManager
  :members:

Network config settings
++++++++++++++++++++++++++++++++++++++++++

+---------------+---------------------------+------------------------------------+
| key           | value example             | Describe                           |
+===============+===========================+====================================+
| method        | ("static"|"dhcp")         |                                    |
+---------------+---------------------------+------------------------------------+
+---------------+---------------------------+------------------------------------+
| **Only required when *method*="static"**                                       |
+---------------+---------------------------+------------------------------------+
| ipaddr        | "192.168.1.2"             | Device ip address (IPv4, str)      |
+---------------+---------------------------+------------------------------------+
| mask          | 24                        | Network mask, int                  |
+---------------+---------------------------+------------------------------------+
| route         | "192.168.1.1"             | Default gateway                    |
+---------------+---------------------------+------------------------------------+
| ns            | ["8.8.8.8"]               | DNS, a list of IPv4 address, str   |
+---------------+---------------------------+------------------------------------+
+---------------+---------------------------+------------------------------------+
| **Only required when config a wifi device**                                    |
+---------------+---------------------------+------------------------------------+
| wifi_mode     | ("host"|"client")         |                                    |
+---------------+---------------------------+------------------------------------+
| ssid          | "A valid SSID"            | A valid ssid to join or hosted     |
+---------------+---------------------------+------------------------------------+
| security      | (None, "WEP", "WPA2-PSK") | Wifi security, None if no security |
+---------------+---------------------------+------------------------------------+
+---------------+---------------------------+------------------------------------+
| **Only required when wifi security="WEP"**                                     |
+---------------+---------------------------+------------------------------------+
| wepkey        | "PASSWORD"                | WEP security password              |
+---------------+---------------------------+------------------------------------+
+---------------+---------------------------+------------------------------------+
| **Only required when wifi security="WPA2-PSK"**                                |
+---------------+---------------------------+------------------------------------+
| psk           | "PASSWORD"                | WPA-PSK security password          |
+---------------+---------------------------+------------------------------------+

Error Handling
++++++++++++++++++++++++++++++++++++++++++

.. autoclass:: fluxclient.device.manager_backends.base.ManagerError
.. autoclass:: fluxclient.device.manager_backends.base.ManagerException

.. sectionauthor:: Cerberus Yagami <cerberus@flux3dp.com>
