
from pkgutil import walk_packages
from setuptools import Extension
import subprocess
import platform
import sys
import os

try:
    from Cython.Distutils import build_ext
    from Cython.Build import cythonize
except ImportError:
    print("""
 ******************************************************
 * Cython must be installed before install fluxclient *
 * Use command `pip install cython` to fix it         *
 ******************************************************
""", file=sys.stderr)
    raise

from fluxclient import __version__ as _VERSION

windows_program_files_sources = None


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


def is_windows():
    return platform.platform().startswith("Windows")


def is_posix():
    return os.name == 'posix'


def is_linux():
    return platform.platform().startswith("Linux")


def is_darwin():
    return platform.platform().startswith("Darwin")


def prepare_setup():
    if not sys.version_info >= (3, 3):
        print("ERROR: fluxclient require Python version grather then 3.3\n",
              file=sys.stderr)
        sys.exit(1)

    # Ensure at correct working directory
    os.chdir(os.path.abspath(os.path.dirname(__file__)))

    options = {}
    return options


def get_version():
    return _VERSION


def get_install_requires():
    return ['setuptools', 'pycrypto', 'pyserial', 'pillow', 'numpy', 'scipy',
            'ecdsa', 'lxml', 'pyasn1']


def get_packages():
    return [name
            for _, name, ispkg in walk_packages(".")
            if name.startswith("fluxclient") and ispkg]


def get_entry_points():
    return {
        "console_scripts": [
            "flux_discover=fluxclient.commands.discover:main",
            "flux_robot=fluxclient.commands.robot:main",
            "flux_upnp=fluxclient.commands.upnp:main",
            "flux_camera=fluxclient.commands.camera:main",
            "flux_scan=fluxclient.commands.scan:main",
            "flux_usb=fluxclient.commands.usb:main",
            "flux_laser=fluxclient.commands.laser:main",
            "flux_g2f=fluxclient.commands.fcode:gcode_2_fcode",
            "flux_f2g=fluxclient.commands.fcode:fcode_2_gcode",
            "flux_exp=fluxclient.commands.experiment_tool:main",
        ]
    }


def get_default_extra_compile_args():
    if is_darwin():
        return ["--stdlib=libc++", "-mmacosx-version-min=10.9"]
    elif is_linux():
        return ["-lstdc++"]
    elif is_windows():
        return []


def create_utils_extentions():
    return [
        Extension(
            'fluxclient.utils._utils',
            sources=[
                "src/utils/utils_module.cpp",
                "src/utils/utils.pyx"],
            language="c++",
            extra_compile_args=get_default_extra_compile_args())
    ]


def create_pcl_extentions():
    # Process include_dirs
    include_dirs = []
    # Process libraries
    libraries = []
    # Process extra_compile_args
    extra_compile_args = get_default_extra_compile_args()
    library_dirs = []

    try:
        if is_posix():
            include_dirs += [locate_includes("eigen3"), ]
            libraries += ["pcl_common", "pcl_octree", "pcl_io", "pcl_kdtree",
                          "pcl_search", "pcl_sample_consensus", "pcl_filters",
                          "pcl_features", "pcl_segmentation", "pcl_surface",
                          "pcl_registration", "pcl_keypoints"]
            if has_package("pcl_common-1.8"):
                include_dirs += [locate_includes("pcl_common-1.8")]
            elif has_package("pcl_common-1.7"):
                include_dirs += [locate_includes("pcl_common-1.7")]
            else:
                raise RuntimeError("Can not locate pcl includes.")
        elif is_windows():
            libraries += ["pcl_common_release", "pcl_octree_release",
                          "pcl_io_release", "pcl_kdtree_release",
                          "pcl_search_release", "pcl_sample_consensus_release",
                          "pcl_filters_release", "pcl_features_release",
                          "pcl_segmentation_release", "pcl_surface_release",
                          "pcl_registration_release", "pcl_keypoints_release"]

            def find_in_program_files(label, names):
                global windows_program_files_sources
                if not windows_program_files_sources:
                    s = []
                    s.append(os.environ["ProgramFiles"])
                    if "ProgramFiles(x86)" in os.environ:
                        s.append(os.environ["ProgramFiles(x86)"])
                    windows_program_files_sources = s

                dirs = []
                for sources in windows_program_files_sources:
                    for name in names:
                        path = os.path.join(sources, name)
                        if os.path.isdir(path):
                            return path
                        else:
                            dirs.append(path)

                raise RuntimeError("Can not find `%s` in any of follow "
                                   "directorys: %s" % (label, dirs))
            try:
                eigen3_dir = os.path.join(
                    find_in_program_files("eigin3", ["Eigen"]), "include")

                flann_dir = os.path.join(
                    find_in_program_files("flann", ["flann"]), "include")
                include_dirs += [
                    os.path.join(eigen3_dir, "include"),
                    os.path.join(flann_dir, "include")]
            except:
                pass

            pcl_dir = find_in_program_files("pcl", ["PCL 1.7.2",
                                                    "PCL 1.7.1"])

            include_dirs += [
                os.path.join(pcl_dir, "lib"),
                os.path.join(pcl_dir, "include", "pcl-1.7"),
                os.path.join(pcl_dir, "3rdParty", "flann", "include"),
                os.path.join(pcl_dir, "3rdParty", "Eigen", "eigen3"),
                os.path.join(pcl_dir, "3rdParty", "Boost", "include",
                             "boost-1_57")]
            library_dirs += [
                os.path.join(pcl_dir, "lib"),
                os.path.join(pcl_dir, "3rdParty", "Boost", "lib")]
        else:
            raise RuntimeError("Unknow platform!!")

    except (RuntimeError, FileNotFoundError) as e:
        import traceback
        print("\033[93m", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        print("""\033[93m
*****************************************************************************
* Can not find pcl libraries, `fluxclient.scanner` and `fluxclient.printer` *
* will not work properly.                                                   *
*****************************************************************************
\033[0m""", file=sys.stderr)
        return []

    extensions = []
    extensions.append(Extension(
        'fluxclient.scanner._scanner',
        sources=[
            "src/scanner/scan_module.cpp",
            "src/scanner/scanner.pyx"],
        language="c++",
        extra_compile_args=extra_compile_args,
        libraries=libraries,
        library_dirs=library_dirs,
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
        library_dirs=library_dirs,
        extra_objects=[],
        include_dirs=include_dirs
    ))

    extensions.append(Extension(
        'fluxclient.utils._utils',
        sources=[
            "src/utils/g2f_module.cpp",
            "src/utils/utils_module.cpp",
            "src/utils/utils.pyx"],
        language="c++",
        extra_compile_args=extra_compile_args,
        libraries=libraries,
        library_dirs=library_dirs,
        extra_objects=[],
        include_dirs=["src/printer/"]
    ))

    return extensions
