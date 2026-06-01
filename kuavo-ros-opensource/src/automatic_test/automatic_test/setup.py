# src/automatic_test/automatic_test/setup.py
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['automatic_test'],
    package_dir={'': 'scripts'}
)

setup(**d)