from setuptools import find_packages
from setuptools import setup

setup(
    name='lift_control',
    version='0.0.0',
    packages=find_packages(
        include=('lift_control', 'lift_control.*')),
)
