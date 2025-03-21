import time
import numpy as np
import threading
import queue
import socket
import medusaiutils
from pythonosc import udp_client

# Global variables
global tstep, client
tstep = 0.004  # Keep timing precise

# Command queues
pluck_queue = queue.Queue()
follow_queue = queue.Queue()

# Snake movement
amps = [0, 5, 0, 15, 5, -30, 0]
phases = [0, 0, 0, 0, 0.5, 0.3, 0]

# Robot configs
pluck_robots = [
    ['192.168.1.236', [0, 40, 0, 117, 0, 44, 0]],
    ['192.168.1.234', [160, 26, 186, 103, 2, 24, -140]],
    ['192.168.1.215', [-9, 5, 7, 133, 1.5, 31, -71]],
    ['192.168.1.208', [0, 18, -185, 89, -5, 22, -10]]
]

follow_robots = [
    ['192.168.1.237', [5, -10, 0, 100, 0, 12, 0]],
    ['192.168.1.244', [-80, -30, -160, 120, 0, 22, 0]],
    ['192.168.1.204', [0, 34, 0, 95, 0, -25, 0]]
]


#  executing trajectories
def runTraj(trajectory, xarms):
    for i in range(len(trajectory[0])):
        start = time.time()
        for num, xarm in enumerate(xarms):
            pos = trajectory[num][i].copy()
            xarm.movexArm(pos)
        t_elapse = time.time() - start
        while t_elapse < tstep:
            time.sleep(0.0001)
            t_elapse = time.time() - start


# follow robots
def makefollowTraj(x, y):
    xlimit = [[120, 330], [330, 475], [330, 500]]
    j3 = [[-25, 67], [-170, -100], [-75, -15]]
    tomove = []
    num = 0
    for xarm in follow_xarms:
        if x <= xlimit[num][0]:
            movej3 = j3[num][0]
        elif x >= xlimit[num][1]:
            movej3 = j3[num][1]
        else:
            movej3 = ((x - xlimit[num][0]) / (xlimit[num][1] - xlimit[num][0]) * (j3[num][1] - j3[num][0])) + j3[num][0]
        num += 1
        curpos = xarm.getAngle1()
        nextspot = curpos.copy()
        nextspot[2] = movej3
        curtraj = xarm.Singlep2ptraj(curpos, nextspot, 2)
        tomove.append(curtraj)
    return tomove


def FollowUser():
    x, y, pluck = follow_queue.queue[-1]

    # create follow trajectory
    tomove = makefollowTraj(x, y)
    runTraj(tomove, follow_xarms)
    follow_queue.queue.clear()

    start = time.time()
    while time.time() - start < 1.5:
        if not follow_queue.empty():
            client.send_message("/Movement", 0)
            x, y, pluck = follow_queue.queue[-1]
            tomove = makefollowTraj(x, y)
            runTraj(tomove, follow_xarms)
            start = time.time()
            follow_queue.queue.clear()
        time.sleep(0.1)

        # Return to snake position
    tomove = []
    for xarm in follow_xarms:
        robottrajirst = xarm.snakebeat1(amps, 5, phases)
        curpos = xarm.getAngle1()
        curtraj = xarm.Singlep2ptraj(curpos, robottrajirst[0], 2)
        tomove.append(curtraj)
    runTraj(tomove, follow_xarms)
    follow_queue.queue.clear()


# Main back robot
def pluck_robomove():
    tomove = []
    for xarm in pluck_xarms:
        robottraj = xarm.snakebeat1(amps, 5, phases)
        tomove.append(robottraj)
    while True:


        for i in range(len(tomove[0])):
            if pluck_queue.qsize() > 0:
                snakePluckTraj = []
                for num, xarm in enumerate(pluck_xarms):
                    robottraj = xarm.snakebeat1(amps, 5, phases)
                    if num == 3:
                        curpos = xarm.getAngle1()
                        snakePluckTraj.append(xarm.Singlep2ptraj(curpos, [22, 30, -196, 93, -4.5, 44.5, -32.5], 2))
                        snakePluckTraj[3] = np.concatenate(
                            (snakePluckTraj[3], xarm.Singlep2ptraj([22, 30, -196, 93, -4.5, 44.5, -32.5],
                                                                   [22, 64.3, -196, 44.8, -4.5, 45.2, 12.7], 2)),
                            axis=0)
                        snakePluckTraj[3] = np.concatenate(
                            (snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 44.8, -4.5, 45.2, 12.7],
                                                                   [22, 64.3, -196, 44.8, -4.5, 58.6, 12.7], 1.5)),
                            axis=0)
                        snakePluckTraj[3] = np.concatenate(
                            (snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 44.8, -4.5, 58.6, 12.7],
                                                                   [22, 64.3, -196, 51.6, -4.5, 45.2, 12.7], 1.5)),
                            axis=0)
                        snakePluckTraj[3] = np.concatenate(
                            (snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 51.6, -4.5, 45.2, 12.7],
                                                                   robottraj[0], 1.5)), axis=0)
                    else:
                        snakePluckTraj.append(tomove[num][i:])
                        remainingNum = len(snakePluckTraj[num])
                        snakePluckTraj[num] = np.concatenate((snakePluckTraj[num], robottraj), axis=0)
                        snakePluckTraj[num] = np.concatenate((snakePluckTraj[num], robottraj), axis=0)

                # Make sure all trajectories are the same length
                pauseNP = np.zeros((len(snakePluckTraj[0]) - len(snakePluckTraj[3]), 7))
                robottraj3 = pluck_xarms[3].snakebeat1(amps, 5, phases)
                pauseNP[:] = robottraj3[0]
                snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], pauseNP), axis=0)

                # Move the robots along the plucking trajectory
                for j in range(len(snakePluckTraj[0])):
                    start = time.time()
                    for num, xarm in enumerate(pluck_xarms):
                        pos = snakePluckTraj[num][j].copy()
                        xarm.movexArm(pos)
                    t_elapse = time.time() - start
                    while t_elapse < tstep:
                        time.sleep(0.0001)
                        t_elapse = time.time() - start

                pluck_queue.queue.clear()
                tomove = []
                for xarm in pluck_xarms:
                    robottraj = xarm.snakebeat1(amps, 5, phases)
                    tomove.append(robottraj)
                break

            # Continue with snake movements
            start = time.time()
            for num, xarm in enumerate(pluck_xarms):
                pos = tomove[num][i].copy()
                xarm.movexArm(pos)
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.001)
                t_elapse = time.time() - start


def follow_robomove():
    tomove = []
    for xarm in follow_xarms:
        robottraj = xarm.snakebeat1(amps, 5, phases)
        tomove.append(robottraj)
    while True:


        for i in range(len(tomove[0])):
            start = time.time()
            for num, xarm in enumerate(follow_xarms):
                pos = tomove[num][i].copy()
                xarm.movexArm(pos)
            t_elapse = time.time() - start

            if follow_queue.qsize() > 0:
                client.send_message("/Movement", 0)
                FollowUser()
                tomove = []
                for xarm in follow_xarms:
                    robottraj = xarm.snakebeat1(amps, 5, phases)
                    tomove.append(robottraj)
                break

            if pluck_queue.qsize() > 0:
                break

            while t_elapse < tstep:
                time.sleep(0.001)
                t_elapse = time.time() - start


# Socket listener
def listen_for_commands():
    while True:
        # Check for pluck commands
        try:
            data, addr = sock_pluck.recvfrom(1024)
            print("Received pluck command")
            pluck_queue.put(1)
        except BlockingIOError:
            pass

        # Check for follow commands
        try:
            data, addr = sock_follow.recvfrom(1024)
            array = np.frombuffer(data, dtype=int)
            print("Follow command:", array[0:2], "Type:", array[2])
            follow_queue.put(array)
        except BlockingIOError:
            pass

        time.sleep(0.001)

    # Initialize robot arrays


pluck_xarms = []
follow_xarms = []

# UDP client for sending messages
IPToSeu = "192.168.1.2"
PORTToSeu = 6005
client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)

# Socket for plucking command
UDP_IP_PLUCK = "192.168.1.1"
UDP_PORT_PLUCK = 5006
sock_pluck = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_pluck.bind((UDP_IP_PLUCK, UDP_PORT_PLUCK))
sock_pluck.setblocking(False)

# Socket for following command
UDP_IP_FOLLOW = "127.0.0.1"
UDP_PORT_FOLLOW = 5005
sock_follow = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follow.bind((UDP_IP_FOLLOW, UDP_PORT_FOLLOW))
sock_follow.setblocking(False)

# Initialize all robots
for robot in pluck_robots:
    pluck_xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for robot in follow_robots:
    follow_xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

# Setup initial positions
for xarm in pluck_xarms:
    IP = xarm.snakebeat1(amps, 3, phases)
    xarm.setupBot(IP[0])

for xarm in follow_xarms:
    IP = xarm.snakebeat1(amps, 3, phases)
    xarm.setupBot(IP[0])

# Start the control and listener threads



# Wait for robots to initialize
input("Press enter when robots stop moving to start script")
threading.Thread(target=pluck_robomove, daemon=True).start()
threading.Thread(target=follow_robomove, daemon=True).start()
threading.Thread(target=listen_for_commands, daemon=True).start()

# Initialize the queues to start the movement
pluck_queue.put(0)
follow_queue.put(np.array([0, 0, 0]))

# Main loop - keep program running
try:
    while True:
    # listen_for_commands()
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Shutting down robot control")
