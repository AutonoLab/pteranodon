#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys


from setuptools import setup
from setuptools.command.test import test as TestCommand

if sys.argv[-1] == 'publish':
    os.system('python setup.py sdist upload')
    sys.exit()

readme = open('README.rst').read()
history = open('HISTORY.rst').read().replace('.. :changelog:', '')


class PyTest(TestCommand):
    def finalize_options(self):
        TestCommand.finalize_options(self)
        self.test_args = []
        self.test_suite = True

    def run_tests(self):
        import pytest
        errcode = pytest.main(self.test_args)
        sys.exit(errcode)


# Specify explicit protobuf dependency for python 3 support
# -> Once 3.0 is released, this should be set to >= 3.0
install_requires = ['protobuf>=3.0.0']

setup(
    name='pygazebo',
    version='4.0.0-2019.07',
    description='Python bindings for the Gazebo multi-robot simulator.',
    long_description=readme + '\n\n' + history,
    author='Justin Davis',
    author_email='jcdavis@mines.edu',
    url='https://github.com/justincdavis/pygazebo',
    packages=[
        'pygazebo',
        'pygazebo.msg',
    ],
    package_dir={'pygazebo': 'pygazebo'},
    include_package_data=True,
    install_requires=install_requires,
    license="Apache License 2.0",
    zip_safe=False,
    keywords='pygazebo',
    classifiers=[
        'Development Status :: 4 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering',
    ],
    tests_require=['pytest', 'mock'],
    extras_require={
        'testing': ['pytest', 'mock'],
        },
    cmdclass={'test': PyTest},
    test_suite='tests',
)
