#!/usr/bin/env python
from setuptools import setup, find_packages
import platform

from fluxclient import VERSION as _VERSION

VERSION = ".".join([str(i) for i in _VERSION])

install_requires = ['setuptools', 'pycrypto']

setup(
    name="fluxclient",
    version=VERSION,
    author="Flux Crop.",
    author_email="cerberus@flux3dp.com",
    description="",
    license="?",
    packages=find_packages(),
    test_suite="tests.main.everything",
    scripts=["bin/flux_discover", "bin/flux_auth", "bin/flux_passwd",
             "bin/flux_config_network"],
    install_requires=install_requires,
)
