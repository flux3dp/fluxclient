Establishing a connection
=================================

Retrieve UUID of FLUX Delta
++++++++++++++++++++++++
::

    from fluxclient.device import DeviceDiscover

    def on_device_discover(device_discover, device, **kw):
        print("Device '%s' found at %s" % (device.name, device.ipaddr))

        my_func_add_rsa_key_to_machine(uuid)

        # Find only one printer in this example
        device_discover.stop()

    device_discover = DeviceDiscover()
    device_discover.discover(on_device_discover)

Adding RSA Key to machine ( first time only )
+++++++++++++++++++++++++
FLUX Device use password and RSA key to authorizing access. At the first time a user connect to device, a RSA key and password is required. After the password is authorized, you can insert your RSA key into device trusted key list.

When creating a DeviceManager instance, call :func:`fluxclient.device.manager.DeviceManager.authorized` to ensure your connection grant permission to operation. If `authorized` return False, you have to call :func:`fluxclient.device.manager.DeviceManager.authorize_with_password` to complete authorize.

::

    from fluxclient.device import discover_device
    from fluxclient.commands.misc import get_or_create_default_key
    client_key = get_or_create_default_key("./sdk_connection.pem")

    def my_func_add_rsa_key_to_machine(uuid):
        my_device = discover_device(uuid)
        device_manager = my_device.manage_device(client_key)

        if device_manager.authorized:
            my_func_connect_robot(my_device)
        else:
            try:
                device_manager.authorize_with_password("your password") #It's the same password you entered in FLUX Studio's configuration page.
                print("Authorized")
                my_func_connect_robot(my_device)
            except ManagerError as e:
                print("Authorize failed: %s" % e)
                raise

.. note:: If you have existing key file, you can use `fluxclient.encrypt.KeyObject.load(file_pointer)`, instead of using fluxclient.commands.misc.get_or_create_default_key

Connecting to 
+++++++++++++++++++++++++

::

    from fluxclient.encrypt import KeyObject
    from fluxclient.robot import connect_robot
    from fluxclient.device import discover_device
    from fluxclient.commands.misc import get_or_create_default_key

    def my_func_connect_robot(device):
        device.connect_robot(client_key)
        maintain_task = device.maintain()
        maintain_task.home()
        maintain_task.quit()


Error Handling
++++++++++++++++++++++++++++++++++++++++++

.. autoclass:: fluxclient.device.manager_backends.ManagerError
.. autoclass:: fluxclient.device.manager_backends.ManagerException

.. sectionauthor:: Cerberus Yagami <cerberus@flux3dp.com>


