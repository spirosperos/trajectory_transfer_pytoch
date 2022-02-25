#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_acin_robot_support'],
    package_dir={'': 'src'},
    requires=['libevalJ','std_msgs', 'rospy', 'sensor_msgs','numpy','sys']
    )

setup(**setup_args)
