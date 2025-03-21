import time
import numpy as np
from collections import deque
# from inputimeout import inputimeout, TimeoutOccurred
import threading
import queue
import math
import socket
import medusaiutils
from pythonosc import udp_client

#Since we run the robot in lots of places I turned this into a function
#This does nothing but maintains time and moves the robot at that time
def runTraj(trajectory):
    for i in range(len(trajectory[0])):
        start = time.time()
        num = 0
        for xarm in xarms:
            pos = trajectory[num][i].copy()
            xarm.movexArm(pos)
            num += 1
        t_elapse = time.time() - start
        while t_elapse < tstep:
            time.sleep(0.0001)
            t_elapse = time.time() - start


def makefollowTraj(x,y):
    #this is the follow stuff elaine made
    xlimit = [[120, 330], [330, 475], [330, 500]]
    j3 = [[-25, 67], [-170, -100], [-75, -15]]
    tomove = []
    num = 0
    for xarm in xarms:
        if x <= xlimit[num][0]:
            movej3 = j3[num][0]
        elif x >= xlimit[num][1]:
            movej3 = j3[num][1]
        else:
            movej3 = ((x - xlimit[num][0]) / (xlimit[num][1] - xlimit[num][0]) * (j3[num][1] - j3[num][0])) + \
                     j3[num][0]
            # print("should stop", movej3)
        num += 1
        curpos = xarm.getAngle1()
        nextspot = curpos.copy()
        nextspot[2] = movej3
        print(curpos)
        # TODO: Gil asks about switching to 1.5 seconds
        curtraj = xarm.Singlep2ptraj(curpos, nextspot, 2)
        # curtraj = xarm.fifth_poly1(curpos[2], movej3, 0, 0, 2)
        tomove.append(curtraj)
    return tomove


def FollowUser():
    # xlimit = [[120, 330], [330, 475]]
    # j3 = [[-25, 67], [-170, -100]]

    #small pause for suspense, can be longer or shorter
    time.sleep(0.25)
    # input("go to position?")
    x, y, pluck = q.queue[-1]
    print(x, y, pluck)

    #creates our follow trajectory
    tomove = makefollowTraj(x,y)
    runTraj(tomove)
    q.queue.clear()

    start = time.time()
    while time.time() - start < 2: # change if it's too slow!!!
        time.sleep(0.1)
        if not q.empty():
            client.send_message("/Movement", 0)
            x, y, pluck = q.queue[-1]
            print(x, y, pluck)
            # creates our follow trajectory
            tomove = makefollowTraj(x, y)
            runTraj(tomove)
            start = time.time()
            q.queue.clear()


    #return to snake position
    tomove = []
    num = 0
    for xarm in xarms:
        robottrajirst = xarm.snakebeat1(amps, 5, phases[num])
        curpos = xarm.getAngle1()
        #We need the first value
        curtraj= xarm.Singlep2ptraj(curpos, robottrajirst[0], 2)
        tomove.append(curtraj)
        num +=1
    for i in range(len(tomove[0])):
        start = time.time()
        num = 0
        for xarm in xarms:
            pos = tomove[num][i].copy()
            xarm.movexArm(pos)
            num += 1
        t_elapse = time.time() - start
        while t_elapse < tstep:
            time.sleep(0.0001)
            t_elapse = time.time() - start
    # input("good?")
    #clear input so that we can reset snaking
    q.queue.clear()



def robomove():
    global amps
    direction = -1
    max = 20

    speed = q.get()
    face = 0
    xlimit = [[120,330],[330,475]]
    j3 = [[-25,67],[-170,-100]]
    percentx = [0]*len(xarms)
    add = 0
    x = 150
    y = 0


    while True:

        ### going up ###
        tomove = []
        num = 0

        # todo: continously having abrupt movement makes the arms stay in position but doesn't trigger sound continuously
        for xarm in xarms:
            #Just do any slow snake movement
            robottraj = xarm.snakebeat1(amps, 5, phases[num])

            num += 1

            tomove.append(robottraj)

        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            for xarm in xarms:
                pos = tomove[num][i].copy()
                # if abs(add) < 20:


                # pos[2] += (face-1) * 0.2
                # print(pos)
                xarm.movexArm(pos)
                num += 1
            t_elapse = time.time() - start
            #instead of checking for movement during generation, we check during execution
            # TODO: I think that once we trigger the abrupt movement, the lights flash, but if we continue triggering it before
            # TODO: the arms reset, the lights do not flash. Maybe we are not sending values during this continuous trigger?
            if q.qsize() > 0:
                print("size of q ", q.qsize())
                # temp_list = list(q.queue)
                # last_item = temp_list[-1] if temp_list else None
                # print("q contents: ", last_item)
                client.send_message("/Movement", 0)
                print("STOP")
                FollowUser()
                q.queue.clear()
                # cuts

                break

            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        # amps = [0, 5, 0, 15, 5, -30, 0]





check = 15

print("DONE", check)

global nbeats
nbeats = 4
global tstep
tstep = 0.004
q = queue.Queue()
IPToSeu = "192.168.1.2"
PORTToSeu = 6005
global client
client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)


UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
test = 130
# while test not in range(60,120):
#     print(test)
#     print("yellow")
#     if test <60:
#         test = test*2
#     else:
#         test = test/2


robots = [['192.168.1.237', [5,-10,0,100,0,12,0]], ['192.168.1.244', [-80,-30,-160,120,0,22,0]], ['192.168.1.204', [0, 34, 0, 95, 0, -25, 0]]] #
xarms = []
# robots = [['192.168.1.237', [5,-10,0,100,0,12,0]]]
basephase = np.array([0, 0, 0, 0, 0.5, 0.3, 0])
speed = 3
amps = [0, 5, 0, 15, 5, -30, 0]
phases = [basephase,basephase+0.25,basephase+0.5]
# amps = [5, 0, 15, 0, 30, 0, 0]
# phases = [0, 0, 0.5, 0, 0.3, 0, 0]
#be


for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))
num = 0
for xarm in xarms:
    IP = xarm.snakebeat1(amps,3,phases[num])
    print(IP[0])
    xarm.setupBot(IP[0])
    num += 1
threading.Thread(target=robomove, daemon=True).start()
input("press enter when robots stop moving to start script")
q.put(0,0)
while True:
    # print("wating")
    data, addr = sock.recvfrom(1024)
    # decoded_tuple = tuple(item.decode('utf-8') for item in data)
    array = np.frombuffer(data, dtype=int)
    print("array from move to max", array[0:2])
    print("Type of movement: ", array[2])
    # print("Received array:", array)
    q.put(array)
    # q.put(int(data.decode('utf-8')))


    # data_list.append(int(data.decode('utf-8')))
    # print(f"Received data: {int(data.decode('utf-8'))}")

    # print(bpm)
    # print(timeb)

    #     except TimeoutOccurred:
    #         exit
    #         engage = False
    #     # input()
    # print("timeout")


