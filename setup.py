#!/usr/bin/env python

### ! DO NOT MANUALLY INVOKE THIS setup.py, THIS IS FOR CATKIN_MAKE TO PROPERLY SET UP THE PACKAGE

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['pymomorphic', 'pymomorphic3'],
     package_dir={'': 'src'},
     install_requires=['csv', 'random', 'rospkg', 'numpy']
)

setup(**setup_args)
