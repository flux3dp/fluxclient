
import warnings
from time import time


class Device(object):
    # Device information
    #   Basic Identify
    _uuid = None
    _serial = None
    _master_key = None

    model_id = None
    version = None

    name = None

    # Network and config related information
    discover_endpoint = None
    ipaddr = None
    last_update = 0

    # Old flux device field
    has_password = None
    slave_timestemp = None
    slave_key = None
    timestemp = None
    timedelta = None

    # Device Status
    _status = None

    def __init__(self, uuid, serial, master_key, disc_ver):
        self._uuid = uuid
        self._serial = serial
        self._master_key = master_key
        self._disc_ver = disc_ver

    def __str__(self):
        return "Device: %s" % self.__m.uuid

    @property
    def uuid(self):
        return self._uuid

    @property
    def serial(self):
        return self._serial

    @property
    def master_key(self):
        return self._master_key

    @property
    def discover_protocol_version(self):
        return self._disc_ver

    def update_status(self, **kw):
        if self._status is None:
            self._status = st = {}
        else:
            st = self._status
        st.update(kw)
        self.last_update = time()

    def connect_robot(self, client_key, port=23811, **kw):
        from fluxclient.robot import connect_robot
        return connect_robot((self.ipaddr, port), client_key,
                             metadata=self.to_old_dict(), **kw)

    def connect_camera(self, client_key, port=23812, **kw):
        from fluxclient.robot import connect_camera
        return connect_camera((self.ipaddr, port), client_key,
                              metadata=self.to_old_dict(), **kw)

    def to_dict(self):
        return {
            "name": self.name,
            "model_id": self.model_id,
            "version": str(self.version),
            "serial": self.serial,
            "master_key": self.master_key,

            "ipaddr": self.ipaddr,
            "discover_endpoint": self.discover_endpoint,
        }

    def to_old_dict(self):
        # with warnings.catch_warnings():
        #     warnings.simplefilter("always")
        warnings.warn("This method will be removed.", DeprecationWarning)
        dataset = {
            "endpoint": self.discover_endpoint,
            "slave_key": self.slave_key,
            "master_ts": self.slave_timestemp,
            "timestemp": self.timestemp,
            "timedelta": self.timedelta,
            "has_password": self.has_password,
        }
        dataset.update(self.to_dict())
        return dataset
