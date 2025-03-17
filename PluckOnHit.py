import numpy as np
import math
import time
import medusaiutils
import socket
import queue
import threading

def pluck():
    points = [[[22, 30, -196, 93, -4.5, 44.5, -32.5], 2, 0], [[22, 64.3, -196, 44.8, -4.5, 45.2, 12.7], 2, 0], [[22, 64.3, -196, 44.8, -4.5, 58.6, 12.7], 1.5, 0], [[22, 64.3, -196, 51.6, -4.5, 45.2, 12.7], 1.5, 0]]

    xarms[0].p2pTraj(points)

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

        for xarm in xarms:
            #Just do any slow snake movement
            robottraj = xarm.snakebeat1(amps, 5, phases)

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
            # TODO: plucking takes 10 seconds.
            if q.qsize() > 0:
                print("START TIME ", time.time())
                pluck()


                # return to snake position
                tomove = []
                num = 0
                for xarm in xarms:
                    robottrajirst = xarm.snakebeat1(amps, 5, phases)
                    curpos = xarm.getAngle1()
                    # We need the first value
                    curtraj = xarm.Singlep2ptraj(curpos, robottrajirst[0], 2)
                    tomove.append(curtraj)
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
                print("END TIME ", time.time())
                # input("good?")
                # clear input so that we can reset snaking

                q.queue.clear()
                # cuts

                break

            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

if __name__ == '__main__':
    robots = [['192.168.1.208', [22, 30, -196, 93, -4.5, 44.5, -32.5]]]

    # TODO: issue when using this script with the SnakeAndFollowAmitToAudio script bc of overlapping IPs??
    UDP_IP = "0.0.0.0"# "192.168.1.2"
    UDP_PORT = 5006
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    canPluck = True

    global nbeats
    nbeats = 4
    global tstep
    tstep = 0.004
    q = queue.Queue()

    xarms = []
    xarmPer = []

    speed = 3
    amps = [0, 5, 0, 15, 5, -30, 0]
    phases = [0, 0, 0, 0, 0.5, 0.3, 0]

    for robot in robots:
        xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

    for xarm in xarms:
        IP = xarm.snakebeat1(amps, 3, phases)
        print(IP[0])
        xarm.setupBot(IP[0])
    threading.Thread(target=robomove, daemon=True).start()
    input("press enter when robots stop moving to start script")
    q.put(0, 0)

    while True:
        # time.sleep(5)
        # data, addr = sock.recvfrom(1024)
        user_input = input("Enter 'p' to pluck robot or 'exit' to quit: ")
        if user_input == 'p':
            q.put(1)

        #add message to q
        # decoded_tuple = tuple(item.decode('utf-8') for item in data)
        """if data and canPluck:
            canPluck = False
            q.put(1)
        canPluck = True
        print(data)
        data = None"""

        # FOR MANUAL PLUCK TESTING PURPOSES



        # array = np.frombuffer(data, dtype=int)


