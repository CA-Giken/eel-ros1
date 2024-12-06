#!/usr/bin/env python

from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["eel_ros1", "eel_bundler"],
    package_dir={'': 'src'},
    scripts=['scripts/main.py']
)

setup(**setup_args)