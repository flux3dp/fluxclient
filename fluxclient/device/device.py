
from uuid import UUID
from time import time
import warnings

from fluxclient.utils.version import StrictVersion
from fluxclient.encryptor import KeyObject

__all__ = ["Device"]

DEVICE_STATUS_CODE = {
    -3: "ST_OCCUPIED_SDK",
    -2: "ST_OCCUPIED_SCAN",
    -1: "ST_OCCUPIED_MAINTAIN",

    0: "ST_IDLE",
    1: "ST_INIT",
    4: "ST_STARTING",
    6: "ST_RESUMING",
    16: "ST_RUNNING",
    18: "ST_RESUMING",
    36: "ST_PAUSED",
    38: "ST_PAUSING",
    48: "ST_PAUSED",
    50: "ST_PAUSING",
    64: "ST_COMPLETED",
    66: "ST_COMPLETING",
    130: "ST_ABORTING",
    128: "ST_ABORTED",
}


class Device(object):
    """A `Device` object is an instance stores device information found from \
:class:`fluxclient.device.DeviceDiscover`"""

    # Device information
    #   Basic Identify
    _uuid = None
    _serial = None
    _master_key = None
    _disc_ver = None

    model_id = None
    version = None

    name = None

    # Network and config related information
    discover_endpoint = None
    ipaddr = None
    last_update = 0

    # Old flux device field
    has_password = None
    slave_timestamp = None
    slave_key = None
    timestamp = None
    timedelta = None

    # Device Status
    _status = None

    @classmethod
    def from_dict(cls, dictobj):
        dv = dictobj["discover_version"]

        uuid = dictobj["uuid"] if isinstance(dictobj["uuid"], UUID) \
            else UUID(hex=dictobj["uuid"])
        version = dictobj["version"] \
            if isinstance(dictobj["version"], StrictVersion) \
            else _to_version_or_none(dictobj["version"])

        device = cls(uuid, dictobj["serial"],
                     _to_keyobj_or_none(dictobj["master_key"]), dv)
        device.name = dictobj["name"]
        device.model_id = dictobj["model_id"]
        device.version = version
        device.ipaddr = dictobj["ipaddr"]
        device.discover_endpoint = tuple(dictobj["discover_endpoint"])
        device.last_update = dictobj["last_update"]
        if dv == 1:
            extend = dictobj["extend_v1"]
            device.has_password = extend["has_password"]
            device.slave_timestamp = extend["slave_timestamp"]
            device.slave_key = _to_keyobj_or_none(extend["slave_key"])
            device.timestamp = extend["timestamp"]
            device.timedelta = extend["timedelta"]
        return device

    def __init__(self, uuid, serial, master_key, disc_ver):
        self._uuid = uuid
        self._serial = serial
        self._master_key = master_key
        self._disc_ver = disc_ver
        self._status = {}

    def __str__(self):
        return "Device: %s" % self.uuid

    @property
    def uuid(self):
        """Unique identification key of the device"""
        return self._uuid

    @property
    def serial(self):
        """Serial number of the device"""
        return self._serial

    @property
    def master_key(self):
        """Master identification key of the device"""
        return self._master_key

    @property
    def discover_protocol_version(self):
        """Discovering protocol version of the device"""
        return self._disc_ver

    def update_status(self, **kw):
        self._status.update(kw)
        self.last_update = time()

    @property
    def status(self):
        """Returns the current status of the device.

        :ivar int st_id: Device status in integer.
        :ivar int st_label: Device status in str.
        :ivar float st_prog: Device running progress in percentage from 0.0 \
to 1.0. Only vaild while running task (st_id > 0).
        :ivar str head_module: Toolhead name installed on device.
        :ivar str error_label: Any error occour during the task.
        :ivar float last_update: Device status update timestamp.

        :rtype: dict
        """
        st_id = self._status.get("st_id")
        st_label = DEVICE_STATUS_CODE.get(st_id, "ST_UNKNOWN")

        return {
            "st_id": st_id,
            "st_label": st_label,
            "st_prog": self._status.get("st_prog"),
            "head_module": self._status.get("head_module"),
            "error_label": self._status.get("error_label"),
            "last_update": self.last_update
        }

    def manage_device(self, client_key, **kw):
        """Creates a device management sesssion.

        :rtype: :class:`fluxclient.device.manager.DeviceManager`
        """

        from fluxclient.device.manager import DeviceManager
        return DeviceManager.from_device(client_key, self)

    def connect_robot(self, client_key, port=23811, **kw):
        """Creates a robot connection

        :rtype: :class:`fluxclient.robot.robot.Robot`
        """

        from fluxclient.robot import FluxRobot
        # TODO
        kw.pop("conn_callback", None)
        return FluxRobot((self.ipaddr, port), client_key, device=self, **kw)

    def connect_camera(self, client_key, port=23812, **kw):
        """Creates a camera connection

        :rtype: :class:`fluxclient.robot.camera.Camera`
        """

        from fluxclient.robot import FluxCamera
        # TODO
        kw.pop("conn_callback", None)
        return FluxCamera((self.ipaddr, port), client_key, device=self, **kw)

    def to_dict(self, serialized=False):
        """Creates a new dictionay store device information

        :param bool serialized: Return a json serializeable dict if \
serialized is True. It is useful if you want to pass flux device information \
over file, socket or other serial devices"""

        return {
            "uuid": str(self._uuid) if serialized else self._uuid,
            "discover_version": self._disc_ver,
            "name": self.name,
            "model_id": self.model_id,
            "version": str(self.version) if serialized else self.version,
            "serial": self.serial,
            "master_key": (self.master_key.public_key_pem.decode()
                           if self.master_key and serialized
                           else self.master_key),

            "ipaddr": self.ipaddr,
            "discover_endpoint": self.discover_endpoint,

            "extend_v1": {
                "has_password": self.has_password,
                "slave_timestamp": self.slave_timestamp,
                "slave_key": (self.slave_key.public_key_pem.decode()
                              if self.slave_key and serialized
                              else self.slave_key),
                "timestamp": self.timestamp,
                "timedelta": self.timedelta
            },
            "last_update": self.last_update
        }

    def to_old_dict(self):
        # with warnings.catch_warnings():
        #     warnings.simplefilter("always")
        warnings.warn("This method will be removed.", DeprecationWarning)
        dataset = {
            "endpoint": self.discover_endpoint,
            "slave_key": self.slave_key,
            "master_ts": self.slave_timestamp,
            "timestamp": self.timestamp,
            "timedelta": self.timedelta,
            "has_password": self.has_password,
        }
        dataset.update(self.to_dict())
        return dataset


def _to_version_or_none(raw_data):
    try:
        if raw_data:
            return StrictVersion(raw_data)
    except (ValueError, TypeError):
        pass


def _to_keyobj_or_none(raw_pem):
    try:
        if raw_pem:
            return KeyObject.load_keyobj(raw_pem)
    except (ValueError, TypeError):
        pass
