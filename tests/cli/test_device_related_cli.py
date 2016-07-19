
import tempfile
import unittest
import json
import os

from fluxclient.commands import discover as discover_cli
from fluxclient.commands import camera as camera_cli
from fluxclient.commands import robot as robot_cli
from fluxclient.commands import scan as scan_cli
from fluxclient.commands import upnp as upnp_cli

from tests.real_device import (
    DEFAULT_UUID,
    DEFAULT_PASSWORD,
    DEFAULT_DEVICE,
    CLIENTKEY,
    CLIENTKEY_FILE)


class DiscoverCliTest(unittest.TestCase):
    def test_discover_with_timeout(self):
        self.assertEqual(discover_cli.main(["-t", "0.05"]), 0)

    @unittest.skipIf(DEFAULT_UUID is None, "Default device not set")
    def test_discover_with_uuid(self):
        swap = tempfile.NamedTemporaryFile()
        discover_cli.main(["-f", "json", "-t", "5", "-o", swap.name,
                          DEFAULT_UUID])
        with open(swap.name, "r") as f:
            result = json.load(f)
            self.assertEqual(len(result), 1)


@unittest.skipIf(DEFAULT_UUID is None, "Default device uuid not set")
@unittest.skipIf(DEFAULT_PASSWORD is None, "Default device password not set")
class UpnpCliTest(unittest.TestCase):
    def test_auth_only(self):
        # Delete all trust keys
        upnp = DEFAULT_DEVICE.manage_device(CLIENTKEY)
        if not upnp.authorized:
            upnp.authorize_with_password(DEFAULT_PASSWORD)
        for item in upnp.list_trust():
            upnp.remove_trust(item["access_id"])

        self.assertEqual(len(upnp.list_trust()), 0)

        # Add singlt key
        upnp_cli.main([DEFAULT_UUID, "--key", CLIENTKEY_FILE.name,
                      "--auth-only", "-p", DEFAULT_PASSWORD])

        # Trusted key should contain only 1 result
        self.assertEqual(len(upnp.list_trust()), 1)


@unittest.skipIf(DEFAULT_UUID is None, "Default device uuid not set")
@unittest.skipIf(DEFAULT_PASSWORD is None, "Default device password not set")
class ControlCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        upnp = DEFAULT_DEVICE.manage_device(CLIENTKEY)
        if not upnp.authorized:
            upnp.authorize_with_password(DEFAULT_PASSWORD)
            upnp.add_trust("UNITTEST", CLIENTKEY)
        upnp.close()

    def test_robot_with_uuid(self):
        robot_cli.main([DEFAULT_UUID, "--key", CLIENTKEY_FILE.name,
                        '--shell',
                        'tests.cli.robot_testcase_helper.with_uuid'])

    def test_robot_with_ipaddr(self):
        robot_cli.main([DEFAULT_DEVICE.ipaddr, "--key", CLIENTKEY_FILE.name,
                        '--shell',
                        'tests.cli.robot_testcase_helper.with_ipaddr'])

    def test_camera_with_uuid(self):
        temp_path = tempfile.mkdtemp()

        try:
            camera_cli.main([DEFAULT_UUID,
                             "--key", CLIENTKEY_FILE.name,
                             "--path", temp_path], oneshot=True)
            filelist = os.listdir(temp_path)
            self.assertEqual(len(filelist), 1)
        finally:
            os.system("rm -rf %s" % temp_path)

    def test_camera_with_ipaddr(self):
        temp_path = tempfile.mkdtemp()

        try:
            camera_cli.main([DEFAULT_DEVICE.ipaddr,
                             "--key", CLIENTKEY_FILE.name,
                             "--path", temp_path], oneshot=True)
            filelist = os.listdir(temp_path)
            self.assertEqual(len(filelist), 1)
        finally:
            os.system("rm -rf %s" % temp_path)

    def test_scan_with_uuid(self):
        temp_path = tempfile.mkdtemp()

        try:
            scan_cli.main([DEFAULT_UUID, "--key", CLIENTKEY_FILE.name,
                           "--path", temp_path, "--auto", "--steps", "1"])
            filelist = os.listdir(temp_path)
            self.assertEqual(len(filelist), 3)
        finally:
            os.system("rm -rf %s" % temp_path)

    def test_scan_with_ipaddr(self):
        temp_path = tempfile.mkdtemp()

        try:
            scan_cli.main([DEFAULT_DEVICE.ipaddr, "--key", CLIENTKEY_FILE.name,
                           "--path", temp_path, "--auto", "--steps", "1"])
            filelist = os.listdir(temp_path)
            self.assertEqual(len(filelist), 3)
        finally:
            os.system("rm -rf %s" % temp_path)
