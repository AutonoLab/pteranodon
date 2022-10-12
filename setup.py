from distutils.core import setup

setup(
    name="pteranodon",
    version="0.0.1",
    description="CSM - Heterogenous Computing Laboratory",
    author="Justin Davis",
    author_email="jcdavis@mines.edu",
    packages=[
        "pteranodon",
        "pteranodon.plugins",
        "pteranodon.plugins.base_plugins",
        "pteranodon.plugins.ext_plugins",
        "pteranodon.plugins.ext_plugins.sensor",
        "csm",
    ],
)
