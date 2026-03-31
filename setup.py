#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['teleop_fetch', 'teleop_fetch.sensors'],
    package_dir={'': 'src'},
)

setup(**d)
