# !/usr/bin/env python3

from pkgutil import walk_packages
from setuptools import Extension
import subprocess
import platform
import sys
import os

try:
    from Cython.Distutils import build_ext
except ImportError:
    print("""
 ******************************************************
 * Cython must be installed before install fluxclient *
 * Use command `pip install cython` to fix it         *
 ******************************************************
""")
    raise

from fluxclient import VERSION as _VERSION


build_ext.user_options.append(
    ("without-pcl", None, "Do not install any extention depend on PCL")
)


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

    options = {"pcl": True}
    if "--without-pcl" in sys.argv:
        sys.argv.pop(sys.argv.index("--without-pcl"))
        options["pcl"] = False

    return options


def get_version():
    return ".".join([str(i) for i in _VERSION])


def get_install_requires():
    return ['setuptools', 'pycrypto', 'pyserial', 'pillow', 'numpy']


def get_packages():
    return [name
            for _, name, ispkg in walk_packages(".")
            if name.startswith("fluxclient") and ispkg]


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

            "flux_laser=fluxclient.commands.laser:main",
            "flux_fcode_conv=fluxclient.commands.fcode:conv"
        ]
    }


def create_common_extentions():
    return []


def create_scanner_extentions():
    try:

        library_dirs = []
        # Process libraries
        libraries = [
            "pcl_common", "pcl_octree", "pcl_io", "pcl_kdtree", "pcl_search",
            "pcl_sample_consensus", "pcl_filters", "pcl_features",
            "pcl_segmentation", "pcl_surface", "pcl_registration", "pcl_keypoints",
            "pcl_tracking", "pcl_recognition", "pcl_outofcore", "pcl_people", ]

        # Process include_dirs
        include_dirs = [
            #locate_includes("eigen3"),
        ]

        # Process extra_compile_args
        extra_compile_args = []
        if platform.platform().startswith("Darwin"):
            extra_compile_args = ["--stdlib=libc++"]
            extra_compile_args += ["-mmacosx-version-min=10.9"]
        elif platform.platform().startswith("Linux"):
            # os.environ['CC'] = 'g++'  # using g++ instead of gcc
            extra_compile_args = ["-lstdc++"]  # flag that tells compiler compile a .cpp file
        elif platform.platform().startswith("Windows"):
            libraries = ["pcl_common_release", "pcl_octree_release", "pcl_io_release", "pcl_kdtree_release", "pcl_search_release",
            "pcl_sample_consensus_release", "pcl_filters_release", "pcl_features_release",
            "pcl_segmentation_release", "pcl_surface_release", "pcl_registration_release", "pcl_keypoints_release",
            "pcl_tracking_release", "pcl_recognition_release", "pcl_outofcore_release", "pcl_people_release"]
            include_dirs += ["C:/Program Files (x86)/Eigen/include"]
            include_dirs += ["C:/Program Files (x86)/flann/include"]
            include_dirs += ["C:/Program Files/PCL 1.7.2/include/pcl-1.7"]
            include_dirs += ["C:/Program Files/PCL 1.7.2/lib"]
            include_dirs += ["C:/local/boost_1_59_0"]
            library_dirs = ["C:\\Program Files\\PCL 1.7.2\\lib"]
        else:
            raise RuntimeError("Unknow platform!!")

        if has_package("pcl_common-1.8"):
            include_dirs += [locate_includes("pcl_common-1.8")]
        elif has_package("pcl_common-1.7"):
            include_dirs += [locate_includes("pcl_common-1.7")]
        else:
            raise RuntimeError("Can not locate pcl includes.")
    except (RuntimeError, FileNotFoundError):
        print("""
*********************************************************************
* Can not build scanner module, maybe you don't have pcl installed, *
* use `--without-pcl` if you don't need pcl functions.              *
*********************************************************************
""")
        raise

    extensions = []
    extensions.append(Extension(
        'fluxclient.scanner._scanner',
        sources=[
            "src/scanner/scan_module.cpp",
            "src/scanner/scanner.pyx"],
        language="c++",
        extra_compile_args=extra_compile_args,
        libraries=libraries,
        library_dirs = library_dirs,
        extra_objects=[],
        include_dirs=include_dirs
    ))
    extensions.append(Extension(
        'fluxclient.printer._printer',
        sources=[
            "src/printer/printer_module.cpp",
            "src/printer/printer.pyx"],
        language="c++",
        extra_compile_args=extra_compile_args,
        libraries=libraries,
        library_dirs = library_dirs,
        extra_objects=[],
        include_dirs=include_dirs
    ))

    return extensions
