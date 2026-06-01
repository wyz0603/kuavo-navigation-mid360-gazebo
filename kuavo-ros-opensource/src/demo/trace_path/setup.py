#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 设置包信息
d = generate_distutils_setup(
    packages=['trace_path'],
    package_dir={'': 'scripts'},
)

setup(**d) 