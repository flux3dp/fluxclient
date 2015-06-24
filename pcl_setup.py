#!/usr/bin/env python3
import platform
import subprocess


def has_package(package_name):
    return subprocess.call(["pkg-config", "--exists", package_name]) == 0


def locate_includes(package_name):
    proc = subprocess.Popen(["pkg-config", "--cflags-only-I", package_name],
                            stdout=subprocess.PIPE)
    if proc.wait() == 0:
        buf = proc.stdout.read().decode("utf8")
        return buf[2:].rstrip()
    else:
        raise RuntimeError("Looking for package error: %s" % package_name)


# EXTRA_COMPILE_ARGS
EXTRA_COMPILE_ARGS = ["--stdlib=libc++"]

if platform.platform().startswith("Darwin"):
    EXTRA_COMPILE_ARGS += ["-mmacosx-version-min=10.9"]
elif platform.platform().startswith("Linux"):
    raise RuntimeError("Not test under linux")
else:
    raise RuntimeError("Unknow platform!!")


# LIBRARYS
LIBRARYS = []
# LIBRARYS += ["boost_system-mt", "boost_filesystem-mt", "boost_thread-mt",
#     "boost_date_time-mt", "boost_iostreams-mt", "boost_serialization-mt",
#     "boost_chrono-mt", ]

LIBRARYS += [
    "pcl_common", "pcl_octree", "pcl_io", "pcl_kdtree", "pcl_search",
    "pcl_sample_consensus", "pcl_filters", "pcl_features", "pcl_segmentation",
    "pcl_surface", "pcl_registration", "pcl_keypoints", "pcl_tracking",
    "pcl_recognition", "pcl_outofcore", "pcl_people",
]
# LIBRARYS += ["pcl_visualization"]


# EXTRA_OBJECTS
EXTRA_OBJECTS = []


# INCLUDE_DIRS
INCLUDE_DIRS = [
    locate_includes("eigen3"),
    # locate_includes("flann"),
]
if has_package("pcl_common-1.8"):
    INCLUDE_DIRS += [locate_includes("pcl_common-1.8")]
elif has_package("pcl_common-1.7"):
    INCLUDE_DIRS += [locate_includes("pcl_common-1.7")]
