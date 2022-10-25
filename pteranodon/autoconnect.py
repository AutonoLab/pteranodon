import asyncio
from typing import List
import pymavlink
import subprocess
import os
import re


def _fetch_possible_serial_devs() -> List[str]:

    blacklisted_devices = ["pmtx", "console", "ttyprintk"]

    serial_devices = os.listdir("/sys/class/tty")

    # Remove blacklisted files
    serial_devices = [dev for dev in serial_devices if dev not in blacklisted_devices]

    # Remove virtual consoles (matches "tty" followed by any number and nothing else)
    serial_devices = [
        dev for dev in serial_devices if re.match("(tty)[0-9]*$", dev) is None
    ]

    serial_paths = ["/dev/" + dev for dev in serial_devices]

    return serial_paths
