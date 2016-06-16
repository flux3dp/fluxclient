
from uuid import UUID
from time import time
import warnings

from fluxclient.encryptor import KeyObject
from fluxclient.utils.version import StrictVersion

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
    """Device is instance store device information found from UpnpDiscover"""

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
    slave_timestemp = None
    slave_key = None
    timestemp = None
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
            device.slave_timestemp = extend["slave_timestemp"]
            device.slave_key = _to_keyobj_or_none(extend["slave_key"])
            device.timestemp = extend["timestemp"]
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
        """Device unique identify"""
        return self._uuid

    @property
    def serial(self):
        """Device serial"""
        return self._serial

    @property
    def master_key(self):
        """Device identify key"""
        return self._master_key

    @property
    def discover_protocol_version(self):
        """Device discover protocol version"""
        return self._disc_ver

    def update_status(self, **kw):
        self._status.update(kw)
        self.last_update = time()

    @property
    def status(self):
        """Device current status.

        :ivar int st_id: Device status in integer.
        :ivar int st_label: Device status in str.
        :ivar float st_prog: Device running progress in percentage from 0.0 \
to 1.0. Only vaild while running task (st_id > 0).
        :ivar str head_module: Toolhead name installed on device.
        :ivar str error_label: Any error occour during the task.
        :ivar float last_update: Device status update timestemp.

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
        """Create a device management sesssion.

        :rtype: :class:`fluxclient.upnp.task.UpnpTask`
        """

        from fluxclient.upnp.task import UpnpTask
        return UpnpTask(self.uuid, client_key, self.ipaddr,
                        device_metadata=self.to_old_dict(), **kw)

    def connect_robot(self, client_key, port=23811, **kw):
        """Create a robot instance

        :rtype: :class:`fluxclient.robot.robot.Robot`
        """
        from fluxclient.robot import connect_robot
        return connect_robot((self.ipaddr, port), client_key,
                             device=self, **kw)

    def connect_camera(self, client_key, port=23812, **kw):
        """Create a camera instance

        :rtype: :class:`fluxclient.robot.camera.Camera`
        """
        from fluxclient.robot import connect_camera
        return connect_camera((self.ipaddr, port), client_key,
                              metadata=self.to_old_dict(), **kw)

    def to_dict(self, serilize=False):
        """Create a new dictionay store divice information

        :param bool serilize: Return a json serializeable dict if serilize is \
True. It is useful if you want to pass flux device information over file, \
socket or other serial devices"""

        return {
            "uuid": str(self._uuid) if serilize else self._uuid,
            "discover_version": self._disc_ver,
            "name": self.name,
            "model_id": self.model_id,
            "version": str(self.version) if serilize else self.version,
            "serial": self.serial,
            "master_key": (self.master_key.public_key_pem.decode()
                           if self.master_key and serilize
                           else self.master_key),

            "ipaddr": self.ipaddr,
            "discover_endpoint": self.discover_endpoint,

            "extend_v1": {
                "has_password": self.has_password,
                "slave_timestemp": self.slave_timestemp,
                "slave_key": (self.slave_key.public_key_pem.decode()
                              if self.slave_key and serilize
                              else self.slave_key),
                "timestemp": self.timestemp,
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
            "master_ts": self.slave_timestemp,
            "timestemp": self.timestemp,
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
