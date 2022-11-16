from pteranodon import SimpleDrone
import sys


hostname = ""
if len(sys.argv) > 1:
    hostname = sys.argv[1]

print(f"Received hostname {hostname}")

test_drone = SimpleDrone(f"udp://{hostname}:14540")
print("Finished-1")
exit(0)
