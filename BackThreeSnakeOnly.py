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
        tomove = []
        num = 0

        for xarm in xarms:
            #Just do any slow snake movement
            robottraj = xarm.snakebeat1(amps, 5, phases)
            # print("XARM START", num, " ", robottraj[0])
            # print("robottraj len ", len(robottraj), np.size(robottraj))
            num += 1
            tomove.append(robottraj)

        # pluckTraj = []
        # pluckTraj.append()

        # end = float(traj[-1])
        # IP = 1
        # TODO: snaking cycle takes 5.2 seconds, plucking takes 9.4 seconds
        # print("START OF NEW SNAKE TRAJ ", time.time())
        for i in range(len(tomove[0])):
            if q.qsize() > 0:
                # TODO: plucking takes longer than a snake cycle. calc snake traj for the max amount of cycles it could
                # TODO: possibly take for the plucking to finish and then pause at the first pos of snaking and wait for
                # TODO: plucking to finish
                # snakingtraj = remainder of current tomove + 2 more tomove
                snakePluckTraj = []
                for num, xarm in enumerate(xarms):
                    robottraj = xarm.snakebeat1(amps, 5, phases)
                    if num == 3:
                        curpos = xarm.getAngle1()
                        snakePluckTraj.append(xarm.Singlep2ptraj(curpos, [22, 30, -196, 93, -4.5, 44.5, -32.5], 2))
                        # print("NP PLUCK", np.size(snakePluckTraj))
                        snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], xarm.Singlep2ptraj([22, 30, -196, 93, -4.5, 44.5, -32.5],
                                                                 [22, 64.3, -196, 44.8, -4.5, 45.2, 12.7], 2)), axis=0)
                        snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 44.8, -4.5, 45.2, 12.7],
                                                                 [22, 64.3, -196, 44.8, -4.5, 58.6, 12.7], 1.5)), axis=0)
                        snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 44.8, -4.5, 58.6, 12.7],
                                                                 [22, 64.3, -196, 51.6, -4.5, 45.2, 12.7], 1.5)), axis=0)
                        # TODO: add another Singlep2ptraj to go to first location of snake
                        snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], xarm.Singlep2ptraj([22, 64.3, -196, 51.6, -4.5, 45.2, 12.7], robottraj[0], 1.5)), axis=0)
                    else:
                        snakePluckTraj.append(tomove[num][i:])
                        # print("NP ELSEEE", snakePluckTraj[num])
                        remainingNum = len(snakePluckTraj[num])
                        # print("NP REMAINING SUM", remainingNum)
                        # print("ROBOTTRAJ ", robottraj)
                        # print("NP REGULAR ROBOT TRAJ", len(robottraj))
                        snakePluckTraj[num] = np.concatenate((snakePluckTraj[num], robottraj), axis=0)
                        snakePluckTraj[num] = np.concatenate((snakePluckTraj[num], robottraj), axis=0)
                        print("WHAT THE LENGTH SHOULD BE", remainingNum + len(robottraj) + len(robottraj))
                        # print("CURR SNAKEPLUCKTRAJ ", snakePluckTraj[num])

                # first check to make sure the original lengths for snakePluckTraj[0] vs. snakePlucKTraj[3] have 0 being longer than 3
                print("LENGTHS ", len(snakePluckTraj[0]), len(snakePluckTraj[3]))
                # print("SNAKE 0 ", snakePluckTraj[0])  # should be a very long 2D array (n, 7)

                pauseNP = np.zeros((len(snakePluckTraj[0]) - len(snakePluckTraj[3]), 7))
                # print(len(pauseNP), pauseNP)
                # TODO: fill np.zeros with the first array in robottraj

                robottraj3 = xarms[3].snakebeat1(amps, 5, phases)

                pauseNP[:] = robottraj3[0]

                snakePluckTraj[3] = np.concatenate((snakePluckTraj[3], pauseNP), axis=0)
                print("NEW LENGTHS ", len(snakePluckTraj[0]), len(snakePluckTraj[3]))

                # MOVE THE XARMS
                for j in range(len(snakePluckTraj[0])):
                    start = time.time()
                    num = 0
                    for xarm in xarms:
                        pos = snakePluckTraj[num][j].copy()
                         #print("XARM ", num, " ", pos)
                        xarm.movexArm(pos)
                        num += 1
                    t_elapse = time.time() - start
                    # instead of checking for movement during generation, we check during execution

                    while t_elapse < tstep:
                        time.sleep(0.0001)
                        t_elapse = time.time() - start


                # MOVE XARMS
                q.queue.clear()
                break

            start = time.time()
            num = 0
            for xarm in xarms:
                pos = tomove[num][i].copy()
                xarm.movexArm(pos)
                num += 1
            t_elapse = time.time() - start
            #instead of checking for movement during generation, we check during execution


            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start



check = 15
print("DONE", check)

global nbeats
nbeats = 4
global tstep
tstep = 0.004
q = queue.Queue()
# IPToSeu = "192.168.1.2"
# PORTToSeu = 6005
global client
# client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)


UDP_IP = "192.168.1.1"
UDP_PORT = 5006
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

robots = [['192.168.1.236', [0, 40, 0, 117, 0, 44, 0]], ['192.168.1.234', [160, 26, 186, 103, 2, 24, -140]], ['192.168.1.215', [-9, 5, 7, 133, 1.5, 31, -71]], ['192.168.1.208', [0, 18, -185, 89, -5, 22, -10]]]
xarms = []

speed = 3
amps = [0, 5, 0, 15, 5, -30, 0]
phases = [0, 0, 0, 0, 0.5, 0.3, 0]
# amps = [5, 0, 15, 0, 30, 0, 0]
# phases = [0, 0, 0.5, 0, 0.3, 0, 0]


for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for xarm in xarms:
    IP = xarm.snakebeat1(amps,3,phases)
    print(IP[0])
    xarm.setupBot(IP[0])
threading.Thread(target=robomove, daemon=True).start()
input("press enter when robots stop moving to start script")
q.put(0, 0)
while True:
    ## MANUAL ##
    """user_input = input("Enter 'p' to pluck robot or 'exit' to quit: ")
    if user_input == 'p':
        q.put(1)"""


    data, addr = sock.recvfrom(1024)
    print("received data from pickup 2")
    q.put(1)
    # print("RECEIVED FROM LAPTOP ", data)




