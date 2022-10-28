class Gazebo:
    """
    A class for interacting with the Gazebo CL interface
    """

    def __init__(self) -> None:
        raise NotImplementedError("Gazebo not implemented")

    def kill_server(self) -> None:
        """
        Kills all Gazebo servers instances
        """
        raise NotImplementedError("Gazebo not implemented")

    def kill_client(self) -> None:
        """
        Kills all Gazebo client instances
        """
        raise NotImplementedError("Gazebo not implemented")

    def kill_all(self) -> None:
        """
        Kills all Gazebo client and server instances
        """
        raise NotImplementedError("Gazebo not implemented")
