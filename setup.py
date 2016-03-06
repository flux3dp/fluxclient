#!/usr/bin/env python3

from setuptools import setup, find_packages
import sys

import setup_utils

options = setup_utils.prepare_setup()

ext_modules = []
ext_modules += setup_utils.create_pcl_extentions()


setup(
    name="fluxclient",
    version=setup_utils.get_version(),
    author="FLUX Inc. Development Team",
    author_email="cerberus@flux3dp.com",
    description="?",
    long_description="\nFLUX Delta user side operating interface",
    keywords=["FLUX"],
    license="?",
    platforms=['Linux', 'Mac OSX', 'Windows'],
    url="https://flux3dp.com/",
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Operating System :: POSIX',
        'Programming Language :: C',
        'Programming Language :: C++',
        'Programming Language :: Cython',
        'Programming Language :: Python :: 3 :: Only',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
    include_package_data=True,
    packages=setup_utils.get_packages(),
    test_suite="tests.main.everything",
    entry_points=setup_utils.get_entry_points(),
    install_requires=setup_utils.get_install_requires(),
    cmdclass={'build_ext': setup_utils.build_ext},
    ext_modules=ext_modules,
)
