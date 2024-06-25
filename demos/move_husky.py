from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot



def move_husky():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    v = 0.5 #m/s
    w = 0.0 #rad /s
    for i in range(30):
        robot.move(v, w)
        simulation.wait(1)

    v = 0.5  # m/s
    w = 0.5  # rad /s
    for i in range(30):
        robot.move(v, w)
        simulation.wait(1)

    v = 0.4  # m/s
    w = -0.5  # rad /s
    for i in range(30):
        robot.move(v, w)
        simulation.wait(1)

    simulation.stop()


def get_lidar_data (lidar, base, vs):
    # get lidar data
    data = lidar.get_laser_data()

    print('Received Laser Data')
    try:
        print(data.shape)
        print(data.dtype)
    except:
        print('Unknown data type')
    return data


if __name__ == "__main__":
    move_husky()
