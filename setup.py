#!/usr/bin/env python3

from setuptools import setup, find_packages
import sys

import setup_utils

options = setup_utils.prepare_setup()

ext_modules = []
if options["pcl"]:
    ext_modules += setup_utils.create_scanner_extentions()


setup(
    name="fluxclient",
    version=setup_utils.get_version(),
    author="Flux Corp.",
    author_email="cerberus@flux3dp.com",
    description="",
    include_package_data=True,
    license="?",
    packages=setup_utils.get_packages(),
    test_suite="tests.main.everything",
    entry_points=setup_utils.get_entry_points(),
    install_requires=setup_utils.get_install_requires(),
    cmdclass={'build_ext': setup_utils.build_ext},
    ext_modules=ext_modules
)
