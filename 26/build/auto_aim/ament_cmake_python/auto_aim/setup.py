from setuptools import find_packages
from setuptools import setup

setup(
    name='auto_aim',
    version='0.0.1',
    packages=find_packages(
        include=('auto_aim', 'auto_aim.*')),
)
