import numpy as np
import math
import time

class robotsUtils:
    def __init__(self, IPadress, initPos, sim = False):
        self.arm = IPadress
        self.initPos =  initPos
        self.tstep = 0.004
        if sim == False:
            from xarm.wrapper import XArmAPI
            self.xArm = XArmAPI(self.arm)
        self.sim = sim
        # self.IPtoSEND = "127.0.0.1" # "192.168.1.50"
        per = 4
        self.ports = tuple(10001 + i for i in range(per)) + tuple(11001 + i for i in range(per)) + tuple(12001 + i for i in range(per)) + tuple(13001 + i for i in range(per))
        #this is indexed at 1
        self.routes = (('melody',) * 8) + (('pitch',) * 8)
        self.client = ...
    
    def setupBot(self):
        self.xArm.set_simulation_robot(on_off=False)
        self.xArm.motion_enable(enable=True)
        self.xArm.clean_warn()
        self.xArm.clean_error()
        self.xArm.set_mode(0)
        self.xArm.set_state(0)
        self.xArm.set_servo_angle(angle=self.initPos, wait=True, speed=20, acceleration=0.5, is_radian=False)
        time.sleep(0.1)
        self.realTimeMode()
        
    
    def realTimeMode(self):
        self.xArm.set_mode(1)
        self.xArm.set_state(0)


    def snakebeat(self,amp, duration,phase):
        t = np.arange(0,2*duration+self.tstep,self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            traj.append([round(-amp[i]*math.cos((math.pi*(q-phase[i]*duration))/duration)+amp[i]+self.initPos[i],4) for q in t])
        traj = np.array(traj)    
        traj = np.transpose(traj)
        return traj

    def movexArm(self, traj):
            if self.sim == False:
                # togo = []
                # for i in range(7):
                #     togo.append(+traj[i])
                self.xArm.set_servo_angle_j(angles=traj, is_radian=False)    
            else:
                print(traj)
