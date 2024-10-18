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
    speeds = np.linspace(5,1,10)
    amps = [0,5,0,15,5,60,0]
    phases = [0,0,0,0,0.5,0.3,0]
    speed = 5
    while True:
        if q.qsize() > 0:
            speed = q.get()
            if speed > len(speeds):
                speed = len(speeds)# item = q.get()
        tomove = []
        for xarm in xarms:
            tomove.append(xarm.snakebeat(amps,speeds[speed],phases))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            for xarm in xarms:
                xarm.movexArm(tomove[num][i])
                num += 1
            t_elapse = time.time()-start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time()-start
        # if direction > 0 :
        #     print("UP",end,IP + direction*end)
        # else :
        #     print("DOWN",end,IP + end + direction*end)
        # direction= -1*direction
            
            
        
        
check = 15        

print("DONE",check)       

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
    

robots = [ ['192.168.1.3',[1,2,3,4,5,6,7]],['192.168.1.3',[1,2,3,4,5,6,7]], 
          ['192.168.1.3',[1,2,3,4,5,6,7]], ['192.168.1.3',[1,2,3,4,5,6,7]], 
          ['192.168.1.3',[1,2,3,4,5,6,7]], ['192.168.1.3',[1,2,3,4,5,6,7]], 
          ['192.168.1.3',[1,2,3,4,5,6,7]] ]
xarms = []
for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0],robot[1],sim=True))
threading.Thread(target=robomove, daemon=True).start()
while True:
    # print("wating")
    data, addr = sock.recvfrom(1024)
    q.put(int(data.decode('utf-8')))
    # data_list.append(int(data.decode('utf-8')))
    # print(f"Received data: {int(data.decode('utf-8'))}")
            
            # print(bpm)
            # print(timeb)
            
            
    #     except TimeoutOccurred:
    #         exit
    #         engage = False
    #     # input()
    # print("timeout")


