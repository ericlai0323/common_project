#!/usr/bin/env python3

from setuptools import setup
from setuptools import find_packages

setup(
    name='odometry_to_tf',
    version='0.0.1',
    packages=find_packages(),
    scripts=['scripts/odometry_to_tf.py'],
    install_requires=['rospy', 'nav_msgs', 'tf'],
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    url='http://www.your-ros-package.org/',
    description='Your ROS package description',
    license='TODO',
    keywords='ROS',
    classifiers=[
        'License :: OSI Approved :: TODO License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development',
    ],
)
