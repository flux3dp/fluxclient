Establishing a connection
=================================

Retrieve UUID of FLUX Delta
++++++++++++++++++++++++
::

    from fluxclient.upnp import UpnpDiscover

    def on_device_discover(upnp_discover, device, **kw):
        print("Device '%s' found at %s" % (device.name, device.ipaddr))

        my_func_add_rsa_key_to_machine(uuid)

        # Find only one printer in this example
        upnp_discover.stop()

    upnp_discover = UpnpDiscover()
    upnp_discover.discover(on_device_discover)

Adding RSA Key to machine ( first time only )
+++++++++++++++++++++++++
FLUX Device use password and RSA key to authorizing access. At the first time a user connect to device, a RSA key and password is required. After the password is authorized, you can insert your RSA key into device trusted key list.

When creating a UpnpTask instance, call :func:`fluxclient.upnp.task.UpnpTask.authorized` to ensure your connection grant permission to operation. If `authorized` return False, you have to call :func:`fluxclient.upnp.task.UpnpTask.authorize_with_password` to complete authorize.

::

    from fluxclient.upnp import discover_device
    from fluxclient.commands.misc import get_or_create_default_key
    client_key = get_or_create_default_key("./sdk_connection.pem")

    def my_func_add_rsa_key_to_machine(uuid):
        my_device = discover_device(uuid)
        upnp_task = my_device.manage_device(client_key)

        if upnp_task.authorized:
            my_func_connect_robot(my_device)
        else:
            try:
                upnp_task.authorize_with_password("your password") #It's the same password you entered in FLUX Studio's configuration page.
                print("Authorized")
                my_func_connect_robot(my_device)
            except UpnpError as e:
                print("Authorize failed: %s" % e)
                raise

.. note:: If you have existing key file, you can use `fluxclient.encrypt.KeyObject.load(file_pointer)`, instead of using fluxclient.commands.misc.get_or_create_default_key

Connecting to 
+++++++++++++++++++++++++

::

    from fluxclient.encrypt import KeyObject
    from fluxclient.robot import connect_robot
    from fluxclient.upnp import discover_device
    from fluxclient.commands.misc import get_or_create_default_key

    def my_func_connect_robot(device):
        device.connect_robot(client_key)
        maintain_task = device.maintain()
        maintain_task.home()
        maintain_task.quit()


Error Handling
++++++++++++++++++++++++++++++++++++++++++

.. autoclass:: fluxclient.upnp.task.UpnpError
.. autoclass:: fluxclient.upnp.task.UpnpException

.. sectionauthor:: Cerberus Yagami <cerberus@flux3dp.com>


