import subprocess


class Tegra:
    """
    A class for starting and parsing information from tegrastats
    """

    def __init__(self, interval=60):
        try:
            subprocess.run(["tegrastats", "--inteval", interval], check=True)
        except subprocess.SubprocessError:
            raise RuntimeError("Tegra could not be started")  # pylint: disable=[W0707]

    @staticmethod
    def battery_5vrail_power() -> int:
        """
        the power measured from the 5 volt rail
        :return: int ; power in watts
        """
        result = subprocess.run(
            [
                "cat",
                "~/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power0_input",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        if result.stderr is not None:
            return int(result.stdout)
        return -1

    @staticmethod
    def battery_gpu_cpu_rail_power() -> int:
        """
        the power measured from the GPU and CPU rail
        :return: int ; power in watts
        """
        result = subprocess.run(
            [
                "cat",
                "~/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power1_input",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        if result.stderr is not None:
            return int(result.stdout)
        return -1

    @staticmethod
    def battery_soc_rail_power() -> int:
        """
        the power measured from the SOC rail
        :return: int ; power in watts
        """
        result = subprocess.run(
            [
                "cat",
                "~/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power2_input",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        if result.stderr is not None:
            return int(result.stdout)
        return -1
