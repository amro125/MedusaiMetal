import time
import numpy as np
from collections import deque
# from inputimeout import inputimeout, TimeoutOccurred
import threading
import queue
import math
import socket
import medusaiutils


# def snakebeat(duration,phase):
#     t = np.arange(0,duration+tstep,tstep)
#     print(t[-1])

#     traj = [round(-0.5*math.cos((math.pi*(q-phase*duration))/duration)+0.5,4) for q in t]
#     return traj

def FollowUser():
    # if q.qsize() > 0: # scary stop
    x, y, pluck = q.queue[-1]
    print(x, y, pluck)
    # print("JFIOEWJFOIWEJFOIWEJFOWIEJFOIWEJF")
    tomove = []
    num = 0

    # TODO: slow down snakebeat but check before
    for xarm in xarms:
        robottrajirst = xarm.snakebeat1(amps, 2.5, phases)
        robottraj = np.full((len(robottrajirst), len(robottrajirst[0])), robottrajirst[0])
        # robottraj = robottrajirst[0]
        # robottraj= robottraj*len(robottrajirst)

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
        xarm.Singlep2ptraj
        trajx = xarm.fifth_poly1(curpos[2], movej3, 0, 0, 2)
        # curpos[2] = trajx[-1]
        newrobottraj = robottraj[:len(trajx)]  # bc the remaining robottraj has empty j3 column
        newrobottraj[:, 2] = trajx
        print("LENGTHS", len(robottraj), len(newrobottraj))
        tomove.append(newrobottraj)
    time.sleep(2)
    tomove = []
    num = 0
    for xarm in xarms:
        
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
    # curpos = []
    # for xarm in xarms:
    #     curpos = xarm.getAngle()

    while True:

        ### going up ###
        tomove = []
        num = 0
        # if q.qsize() > 0: # scary stop
        #     x, y, pluck = q.queue[-1]
        #     print(x, y, pluck)
        #     # print("JFIOEWJFOIWEJFOIWEJFOWIEJFOIWEJF")
        #
        #     # TODO: slow down snakebeat but check before
        #     for xarm in xarms:
        #         robottrajirst = xarm.snakebeat1(amps, 2.5, phases)
        #         robottraj = np.full((len(robottrajirst), len(robottrajirst[0])), robottrajirst[0])
        #         # robottraj = robottrajirst[0]
        #         # robottraj= robottraj*len(robottrajirst)
        #
        #         if x <= xlimit[num][0]:
        #             movej3 = j3[num][0]
        #         elif x >= xlimit[num][1]:
        #             movej3 = j3[num][1]
        #         else:
        #             movej3 = ((x - xlimit[num][0]) / (xlimit[num][1] - xlimit[num][0]) * (j3[num][1] - j3[num][0])) + \
        #                      j3[num][0]
        #             # print("should stop", movej3)
        #         num += 1
        #         curpos = xarm.getAngle1()
        #         print(curpos)
        #         trajx = xarm.fifth_poly1(curpos[2], movej3, 0, 0, 2)
        #         # curpos[2] = trajx[-1]
        #         newrobottraj = robottraj[:len(trajx)]  # bc the remaining robottraj has empty j3 column
        #         newrobottraj[:, 2] = trajx
        #         print("LENGTHS", len(robottraj), len(newrobottraj))
        #         tomove.append(newrobottraj)
        #     q.queue.clear()

        # else:
        for xarm in xarms:
            robottraj = xarm.snakebeat1(amps, 2.5, phases)

            # if x <= xlimit[num][0]:
            #     movej3 = j3[num][0]
            # elif x >= xlimit[num][1]:
            #     movej3 = j3[num][1]
            # else:
            #     movej3 =((x - xlimit[num][0]) / (xlimit[num][1] - xlimit[num][0]) * (j3[num][1]-j3[num][0])) + j3[num][0]
                # print("!Q!!!!!!!!!!!!!!!!!!!!!!!!!!!",movej3)
            num += 1
            # curpos = xarm.getAngle1()
            # print(curpos)
            # trajx = xarm.fifth_poly1(curpos[2],movej3,0,0,2.5)
            # curpos[2] = trajx[-1]
            # robottraj[:,2] = trajx
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
            if q.qsize() > 0:
                print("STOP")
                q.queue.clear()
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


robots = [['192.168.1.237', [5,-10,0,100,0,12,0]], ['192.168.1.244', [-80,-30,-160,120,0,22,0]] ]
xarms = []
# robots = [['192.168.1.237', [5,-10,0,100,0,12,0]]]

speed = 3
amps = [0, 5, 0, 15, 5, -30, 0]
phases = [0, 0, 0, 0, 0.5, 0.3, 0]
# amps = [5, 0, 15, 0, 30, 0, 0]
# phases = [0, 0, 0.5, 0, 0.3, 0, 0]
#be


for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for xarm in xarms:
    IP = xarm.snakebeat1(amps,3,phases)
    print(IP[0])
    xarm.setupBot(IP[0])
threading.Thread(target=robomove, daemon=True).start()
input("press enter when robots stop moving to start script")
q.put(0,0)
while True:
    # print("wating")
    data, addr = sock.recvfrom(1024)
    # decoded_tuple = tuple(item.decode('utf-8') for item in data)
    array = np.frombuffer(data, dtype=int)
    print("array from move to max", array[0:2])
    print("Do I pluck:", array[2])
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


