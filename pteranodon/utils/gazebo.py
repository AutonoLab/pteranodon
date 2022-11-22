import subprocess


def kill_gz_server() -> None:
    """Kill all gzserver processes."""
    subprocess.run(["killall", "-9", "gzserver"])

def kill_gz_client() -> None:
    """Kill all gzclient processes."""
    subprocess.run(["killall", "-9", "gzclient"])

def kill_gazebo() -> None:
    """Kill all gzserver and gzclient processes."""
    kill_gz_server()
    kill_gz_client()
    