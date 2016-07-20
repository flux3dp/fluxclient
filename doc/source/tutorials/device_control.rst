Full Control
=================================
This guide will show you how to control every aspects of the machine, you may need to understand the connection protocol this time.

Retrieving the UUID
++++++++++++++++++++++++
FLUXClient uses UUID to determine different machine. You can use :class:`fluxclient.upnp.UpnpDiscover` to find all machines on the local network.

.. note::
    You may use :func:`fluxclient.upnp.UpnpDiscover.poke` to force a communication if your delta does not show up in discovering list at the beginning.

::

    from fluxclient.upnp import UpnpDiscover

    def my_func_device_discover(upnp_discover, device, **kw):
        print("Device '%s' found at %s" % (device.name, device.ipaddr))

        # Test if the machine authorized
        my_func_auth_device(device)

        # Find only one printer in this example
        upnp_discover.stop()

    upnp_discover = UpnpDiscover()
    upnp_discover.discover(my_func_device_discover)

Adding RSA key to the machine ( first time )
+++++++++++++++++++++++++
FLUX Delta use password and RSA key to authorize user permission. At the first time a user connect to a device, an RSA key and password is required. After confirming the correctness of the password, you can then insert your RSA key into device trusted key list.

When creating a UpnpTask instance, call :func:`fluxclient.upnp.task.UpnpTask.authorized` to ensure your connection grant permission to operation. If `authorized` return False, you have to call :func:`fluxclient.upnp.task.UpnpTask.authorize_with_password` to complete authorize.

.. note:: :class:`fluxclient.upnp.device.Device` represents a public-broadcasted information of a machine. So you can fetch the status of the device (like WORKING, COMPLETED ), without getting authorized.

::

    from fluxclient.upnp import discover_device, UpnpError
    from fluxclient.commands.misc import get_or_create_default_key

    my_rsakey = get_or_create_default_key("./sdk_connection.pem")

    def my_func_auth_device(my_device):
        upnp_task = my_device.manage_device(my_rsakey)

        if upnp_task.authorized:
            my_func_connect_robot(my_device)
        else:
            try:
                upnp_task.authorize_with_password("your password") #It's the same password you entered in FLUX Studio's configuration page.
                upnp_task.add_trust("my_public_key", my_rsakey.public_key_pem.decode())
                print("Authorized")
                my_func_connect_robot(my_device)
            except UpnpError as e:
                print("Authorization failed: %s" % e)
                raise

.. note:: If you have existing key file, you can use `fluxclient.encrypt.KeyObject.load(file_pointer)`, instead of using fluxclient.commands.misc.get_or_create_default_key

Establishing a realtime connection
+++++++++++++++++++++++++
After adding RSA Key to FLUX Delta, you can now feel safe to establish a new connection. :class:`fluxclient.robot.robot.FluxRobot` repesents a realtime connection with the machine.
::

    from fluxclient.encryptor import KeyObject
    from fluxclient.upnp import discover_device

    my_rsakey = get_or_create_default_key("./sdk_connection.pem")

    def my_func_connect_robot(my_device):
        robot = my_device.connect_robot(my_rsakey)
        maintain_task = robot.maintain()
        maintain_task.home()
        maintain_task.quit()


Complete example code
+++++++++++++++++++++++++

::

    from fluxclient.upnp import UpnpDiscover
    from fluxclient.encryptor import KeyObject
    from fluxclient.upnp import discover_device, UpnpError
    from fluxclient.commands.misc import get_or_create_default_key

    my_rsakey = get_or_create_default_key("./sdk_connection.pem")

    def my_func_device_discover(upnp_discover, device, **kw):
        print("Device '%s' found at %s" % (device.name, device.ipaddr))

        # Test if the machine authorized
        my_func_auth_device(device)

        # Find only one printer in this example
        upnp_discover.stop()

    def my_func_auth_device(my_device):
        upnp_task = my_device.manage_device(my_rsakey)

        if upnp_task.authorized:
            my_func_connect_robot(my_device)
        else:
            try:
                upnp_task.authorize_with_password("your password") #It's the same password you entered in FLUX Studio's configuration page.
                upnp_task.add_trust("my_public_key", my_rsakey.public_key_pem.decode())
                print("Authorized")
                my_func_connect_robot(my_device)
            except UpnpError as e:
                print("Authorization failed: %s" % e)
                raise

    def my_func_connect_robot(my_device):
        robot = my_device.connect_robot(my_rsakey)
        maintain_task = robot.maintain()
        maintain_task.home()
        maintain_task.quit()

    upnp_discover = UpnpDiscover()
    upnp_discover.discover(my_func_device_discover)


More commands
+++++++++
Kindly check documentation of :class:`fluxclient.robot.robot.FluxRobot`.