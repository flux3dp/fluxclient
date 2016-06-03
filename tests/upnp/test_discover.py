
from uuid import UUID
import unittest
import logging

from fluxclient.upnp import UpnpDiscover

DEFAULT_UUID = UUID(hex="6b75726f6e656b6f77616d6f656d6f65")
logger = logging.getLogger(__name__)


class TestDiscover(unittest.TestCase):
    def test_filter(self):
        bad_uuid = UUID(hex="656368696e616e6f6861696b656e6169")

        u = UpnpDiscover(uuid=DEFAULT_UUID)
        ret = u.source_filter(DEFAULT_UUID, ("192.168.1.1", 3128))
        self.assertTrue(ret)
        ret = u.source_filter(bad_uuid, ("192.168.1.1", 3128))
        self.assertFalse(ret)

        u = UpnpDiscover(device_ipaddr="192.168.2.1")
        ret = u.source_filter(DEFAULT_UUID, ("192.168.2.1", 3128))
        self.assertTrue(ret)
        ret = u.source_filter(DEFAULT_UUID, ("192.168.1.1", 3128))
        self.assertFalse(ret)

