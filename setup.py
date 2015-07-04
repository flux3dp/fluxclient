#!/usr/bin/env python3

from setuptools import setup, find_packages

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

import setup_utils

setup_utils.prepare_setup()
VERSION = setup_utils.get_version()

setup(
    name="fluxclient",
    version=VERSION,
    author="Flux Crop.",
    author_email="cerberus@flux3dp.com",
    description="",
    license="?",
    packages=find_packages(),
    test_suite="tests.main.everything",
    entry_points=setup_utils.get_entry_points(),
    install_requires=setup_utils.get_install_requires(),
    cmdclass = {'build_ext': build_ext},
    ext_modules=[
        setup_utils.create_scanner_extention()
    ]
)
