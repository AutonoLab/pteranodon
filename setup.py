from skbuild import setup

setup(
    packages=[
        "pteranodon",
        "pteranodon.utils",
        "pteranodon.tools",
        "pteranodon.ext",
        "pteranodon.implementations",
        "pteranodon.implementations.hardware",
        "pteranodon.implementations.networks",
        "pteranodon.plugins",
        "pteranodon.plugins.base_plugins",
        "pteranodon.plugins.custom_plugins",
        "pteranodon.plugins.extension_plugins",
        "pteranodon.plugins.extension_plugins.config",
        "pteranodon.plugins.extension_plugins.power",
        "pteranodon.plugins.extension_plugins.relative",
        "pteranodon.plugins.extension_plugins.sensor",
        ],
    package_dir={"": ""},
    cmake_install_dir="pteranodon/ext/",
    cmake_source_dir="",
    python_requires=">=3.8",
)



# [project]
# name = "pteranodon"
# version = "0.0.1.1"
# description = "A framework for simplistic control of autonomous UAVs"
# license = "LICENSE"
# readme = "README.md"
# classifiers = [
#     "Programming Language :: Python :: 3",
#     "License :: OSI Approved :: MIT License",
#     "Operating System :: OS Independent",
# ]
# dependencies = [
#     "mavsdk>=1.4.0",
#     "numpy>=1.23.0",
#     "pymavlink>=2.4.37",
#     "grpcio>=1.47.0",
#     "pyserial>=3.5",
# ]

# [project.urls]
# "Homepage" = "https://github.com/AutonoLab/pteranodon"
# "Bug Tracker" = "https://github.com/AutonoLab/pteranodon/issues"
