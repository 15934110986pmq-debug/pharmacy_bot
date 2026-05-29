#!/usr/bin/env python3
from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup
from Cython.Build import cythonize
import numpy

d = generate_distutils_setup(
    packages=['jetarm_kinematics'],
    package_dir={'': 'src'},
 #   ext_modules = cythonize('src/jetarm_kinematics/ik.pyx'),
    include_dirs=[numpy.get_include()],
    package_data = {
        'jetarm_kinematics': ['*.so'],
        }
 )

setup(**d)
