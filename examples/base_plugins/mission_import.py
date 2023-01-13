from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    mission_import_data = drone.mission_raw.import_qgroundcontrol_mission(
        "examples/base_plugins/example-mission.plan"
    )
    drone.logger.info(
        f"{len(mission_import_data.mission_items)} mission items imported"
    )
    drone.mission_raw.upload_mission(mission_import_data.mission_items)

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
