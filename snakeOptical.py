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


def robomove():
    direction = -1
    max = 20

    speed = q.get()
    face = 0
    add = 0
    while True:

        ### going up ###
        if q.qsize() > 0:
            input= q.queue[-1]
            speed = input[0]
            face = input[1]

            goal = max*(face-1)

            # speed = q.queue[-1]

            print("q speed",speed)
            q.queue.clear()
            if speed > len(speeds)-1:
                speed = len(speeds)-1  # item = q.get()
        tomove = []
        for xarm in xarms:
            tomove.append(xarm.snakebeathalf(amps, speeds[speed], phases))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            for xarm in xarms:
                pos = tomove[num][i].copy()
                # if abs(add) < 20:


                # pos[2] += (face-1) * 0.2

                xarm.movexArm(pos)
                num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        ### going down ####
        #[x+1 for x in mylist]
        if q.qsize() > 0:
            input = q.queue[-1]
            speed = input[0]
            face = input[1]
            # speed = q.queue[-1]
            print("q speed", speed)
            q.queue.clear()
            if speed > len(speeds) - 1:
                speed = len(speeds) - 1  # item = q.get()
        tomove = []
        for xarm in xarms:
            tomove.append(xarm.snakebeathalf(amps, speeds[speed],[x+1 for x in phases]))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            for xarm in xarms:
                xarm.movexArm(tomove[num][i])
                num += 1
            t_elapse = time.time() - start
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


robots = [['192.168.1.242', [5,-10,0,100,0,12,0]], ['192.168.1.244', [98,31,25,110,0,22,0]] ]
xarms = []

speeds = np.linspace(3, 0.5, 20)
# amps = [0, 5, 0, 15, 5, -30, 0]
# phases = [0, 0, 0, 0, 0.5, 0.3, 0]
amps = [5, 0, 15, 0, 30, 0, 0]
phases = [0, 0, 0.5, 0, 0.3, 0, 0]
#be
for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for xarm in xarms:
    IP = xarm.snakebeat(amps,speeds[0],phases)
    print(IP[0])
    xarm.setupBot(IP[0])
threading.Thread(target=robomove, daemon=True).start()
input("press enter when robots stop moving to start script")
q.put(0)
while True:
    # print("wating")
    data, addr = sock.recvfrom(1024)
    array = np.frombuffer(data,dtype = int)
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


