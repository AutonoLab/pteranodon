from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    print("Setting mode to 'PHOTO'")
    drone.camera.set_mode(drone.camera.mode.PHOTO)

    print("Taking a photo")
    drone.camera.take_photo()


if __name__ == "__main__":
    run()
