from mavsdk.mission import MissionItem, MissionPlan

from pteranodon import SimpleDrone


def run():
    drone = SimpleDrone("udp://:14540")

    mission_items = [MissionItem(47.398039859999997,
                                 8.5455725400000002,
                                 25,
                                 10,
                                 True,
                                 float('nan'),
                                 float('nan'),
                                 MissionItem.CameraAction.NONE,
                                 float('nan'),
                                 float('nan'),
                                 float('nan'),
                                 float('nan'),
                                 float('nan')), MissionItem(47.398036222362471,
                                                            8.5450146439425509,
                                                            25,
                                                            10,
                                                            True,
                                                            float('nan'),
                                                            float('nan'),
                                                            MissionItem.CameraAction.NONE,
                                                            float('nan'),
                                                            float('nan'),
                                                            float('nan'),
                                                            float('nan'),
                                                            float('nan')), MissionItem(47.397825620791885,
                                                                                       8.5450092830163271,
                                                                                       25,
                                                                                       10,
                                                                                       True,
                                                                                       float('nan'),
                                                                                       float('nan'),
                                                                                       MissionItem.CameraAction.NONE,
                                                                                       float('nan'),
                                                                                       float('nan'),
                                                                                       float('nan'),
                                                                                       float('nan'),
                                                                                       float('nan'))]

    mission_plan = MissionPlan(mission_items)
    drone.logger.info("-- Uploading mission")
    drone.mission_raw.upload_mission(mission_plan)

    drone.logger.info("-- Arming")
    drone.action.arm()

    drone.logger.info("-- Starting mission")
    drone.mission_raw.start_mission()

    drone.wait_until_queue_empty()
    drone.stop()


if __name__ == "__main__":
    run()
