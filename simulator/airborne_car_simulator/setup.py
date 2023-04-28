from distutils.core import setup
from os.path import isdir
from itertools import product

all_packages = ['airborne_car_simulator']
packages = list(filter(isdir, all_packages))

setup(
    name='ese615',
    packages=packages,
    version='0.1',
    install_requires=[
            'pyyaml',
            'opencv-python',
            'matplotlib == 3.7.1',
            'numpy',
            'scipy'])
