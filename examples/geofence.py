from mavsdk.geofence import Point, Polygon

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    drone.logger.info("Fetching home location coordinates...")
    latitude = drone.telemetry.home.latitude_deg
    longitude = drone.telemetry.home.longitude_deg

    p1 = Point(latitude - 0.0001, longitude - 0.0001)
    p2 = Point(latitude + 0.0001, longitude - 0.0001)
    p3 = Point(latitude + 0.0001, longitude + 0.0001)
    p4 = Point(latitude - 0.0001, longitude + 0.0001)

    # Create a polygon object using your points
    polygon = Polygon([p1, p2, p3, p4], Polygon.FenceType.INCLUSION)

    # Upload the geofence to your vehicle
    drone.logger.info("Uploading geofence...")
    drone.geofence.upload_geofence([polygon])

    drone.stop()


if __name__ == "__main__":
    run()
