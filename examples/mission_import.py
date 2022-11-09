from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    mission_import_data = drone.mission_raw.import_qgroundcontrol_mission("example-mission.plan")
    print(f"{len(mission_import_data.mission_items)} mission items imported")
    drone.mission_raw.upload_mission(mission_import_data.mission_items)


if __name__ == "__main__":
    run()

