
from setuptools import Extension
import subprocess
import platform
import sys
import os

from fluxclient import VERSION as _VERSION


# Base method to find package in system
def has_package(package_name):
    return subprocess.call(["pkg-config", "--exists", package_name]) == 0


# Base method to find library include files in system
def locate_includes(package_name):
    proc = subprocess.Popen(["pkg-config", "--cflags-only-I", package_name],
                            stdout=subprocess.PIPE)
    if proc.wait() == 0:
        buf = proc.stdout.read().decode("utf8")
        return buf[2:].rstrip()
    else:
        raise RuntimeError("Looking for package error: %s" % package_name)


def prepare_setup():
    if not sys.version_info >= (3, 3):
        print("ERROR: fluxclient require Python version grather then 3.3\n")
        sys.exit(1)

    # Ensure at correct working directory
    os.chdir(os.path.abspath(os.path.dirname(__file__)))


def get_version():
    return ".".join([str(i) for i in _VERSION])


def get_install_requires():
    return ['setuptools', 'pycrypto', 'pyserial', 'pillow']


def get_entry_points():
    return {
        "console_scripts": [
            "flux_discover=fluxclient.commands.discover:main",
            "flux_passwd=fluxclient.commands.passwd:main",
            "flux_auth=fluxclient.commands.auth:main",
            "flux_config_network=fluxclient.commands.config_network:main",
            "flux_robot=fluxclient.commands.robot:main",
            "flux_scan=fluxclient.commands.scan:main",
            "flux_usb=fluxclient.commands.usb:main",

            "flux_laser=fluxclient.commands.laser_patten:main"
        ],
    }


def create_scanner_extention():
    # Process extra_compile_args
    extra_compile_args = ["--stdlib=libc++"]

    if platform.platform().startswith("Darwin"):
        extra_compile_args += ["-mmacosx-version-min=10.9"]
    elif platform.platform().startswith("Linux"):
        raise RuntimeError("Not test under linux")
    else:
        raise RuntimeError("Unknow platform!!")

    # Process libraries
    libraries = [
        "pcl_common", "pcl_octree", "pcl_io", "pcl_kdtree", "pcl_search",
        "pcl_sample_consensus", "pcl_filters", "pcl_features",
        "pcl_segmentation", "pcl_surface", "pcl_registration", "pcl_keypoints",
        "pcl_tracking", "pcl_recognition", "pcl_outofcore", "pcl_people", ]

    # Process include_dirs
    include_dirs = [
        locate_includes("eigen3"),
    ]
    if has_package("pcl_common-1.8"):
        include_dirs += [locate_includes("pcl_common-1.8")]
    elif has_package("pcl_common-1.7"):
        include_dirs += [locate_includes("pcl_common-1.7")]

    return Extension(
        'fluxclient.scanner._scanner',
        sources=[
            "src/scanner/scan_module.cpp",
            "src/scanner/scanner.pyx"],
        language="c++",
        extra_compile_args=extra_compile_args,
        libraries=libraries,
        extra_objects=[],
        include_dirs=include_dirs
    )

