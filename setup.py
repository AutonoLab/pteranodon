from distutils.core import setup

setup(
    name="pteranodon",
    version="0.0.1",
    description="CSM - Heterogenous Computing Laboratory",
    author="Justin Davis",
    author_email="jcdavis@mines.edu",
    packages=[
        "pteranodon",
        "pteranodon.utils",
        "pteranodon.plugins",
        "pteranodon.plugins.base_plugins",
        "pteranodon.plugins.extension_plugins",
        "pteranodon.plugins.extension_plugins.sensor",
        "pteranodon.plugins.extension_plugins.relative",
        "pteranodon.plugins.extension_plugins.power",
        "pteranodon.plugins.custom_plugins",
    ],
)
