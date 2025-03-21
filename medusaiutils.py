import numpy as np
import math
import time

global stepT
stepT = 0.004

class robotsUtils:
    def __init__(self, IPadress, initPos, sim=False):
        self.arm = IPadress
        self.initPos = initPos
        # global startP
        # startP = initPos
        self.tstep = 0.004
        if sim == False:
            from xarm.wrapper import XArmAPI
            self.xArm = XArmAPI(self.arm)
        self.sim = sim
        # self.IPtoSEND = "127.0.0.1" # "192.168.1.50"
        per = 4
        self.ports = tuple(10001 + i for i in range(per)) + tuple(11001 + i for i in range(per)) + tuple(
            12001 + i for i in range(per)) + tuple(13001 + i for i in range(per))
        # this is indexed at 1
        self.routes = (('melody',) * 8) + (('pitch',) * 8)
        self.client = ...

    def setupBot(self,IP = 0):
        if not self.sim:
            self.xArm.set_simulation_robot(on_off=False)
            self.xArm.motion_enable(enable=True)
            self.xArm.clean_warn()
            self.xArm.clean_error()
            self.xArm.set_mode(0)
            self.xArm.set_state(0)
            self.xArm.set_servo_angle(angle=IP, wait=True, speed=20, acceleration=0.5, is_radian=False)
            time.sleep(0.1)
            self.realTimeMode()

    def shutoff(selfself):
        self.xArm.motion_enable(enable=True)

    def realTimeMode(self):
        self.xArm.set_mode(1)
        self.xArm.set_state(0)


    # def onetimesnake(self, amp, t, phase):

    def snakebeat(self, amp, duration, phase):
        t = np.arange(0, 2 * duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            traj.append([round(
                -amp[i] * math.cos((math.pi * (q - phase[i] * duration)) / duration)  + self.initPos[i], 4) for
                         q in t]) # add + amp[i] for cosine
        traj = np.array(traj)
        traj = np.transpose(traj)
        return traj

    def snakebeat1(self, amp, duration, phase):
        t = np.arange(0,  duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            traj.append([round(
                -amp[i] * math.cos((math.pi * 2*(q - phase[i] * duration)) / duration)  + self.initPos[i], 4) for
                         q in t]) # add + amp[i] for cosine
        traj = np.array(traj)
        traj = np.transpose(traj)
        return traj

    def snakebeathalf(self, amp, duration, phase):
        t = np.arange(0,  duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            # if i % 2 == 0:
            #     taj.append(2)
            # else:
            traj.append([round(
                -amp[i] * math.sin((math.pi * (q - phase[i] * duration)) / duration)  + self.initPos[i], 4) for
                         q in t])

        traj = np.array(traj)
        traj = np.transpose(traj)
        return traj

    def snakebeathalfcos(self, amp, duration, phase):
        t = np.arange(0,  duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            # if i % 2 == 0:
            #     taj.append(2)
            # else:
            traj.append([round(
                -amp[i] * math.cos((math.pi * (q - phase[i] * duration)) / duration) + amp[i]  + self.initPos[i], 4) for
                         q in t])

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

    def movexArmSpeed(self, traj, speed):
        if self.sim == False:
            self.xArm.set_servo_angle(angle=IP, wait=True, speed=20, acceleration=0.5, is_radian=False)
        else:
            print(traj)

    def getAngle(self):
        return self.xArm.get_servo_angle()

    def getAngle1(self):
        return self.xArm.get_servo_angle()[1]

    def fifth_poly(self, q_i, q_f, v_i, vf, t):
        # time/0.005
        traj_t = np.arange(0, t, stepT)
        dq_i = v_i
        dq_f = vf
        ddq_i = 0
        ddq_f = 0
        a0 = q_i
        a1 = dq_i
        a2 = 0.5 * ddq_i
        a3 = 1 / (2 * t ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * t - (3 * ddq_f - ddq_i) * t ** 2)
        a4 = 1 / (2 * t ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * t + (3 * ddq_f - 2 * ddq_i) * t ** 2)
        a5 = 1 / (2 * t ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * t - (ddq_f - ddq_i) * t ** 2)
        traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
        for i in range(len(traj_pos)):
            traj_pos[i] = round(traj_pos[i], 5)
        return traj_pos

    def fifth_poly1(self, q_i, q_f, v_i, vf, t):
        # time/0.005
        traj_t = np.arange(0, t, self.tstep)
        dq_i = v_i
        dq_f = vf
        ddq_i = 0
        ddq_f = 0
        a0 = q_i
        a1 = dq_i
        a2 = 0.5 * ddq_i
        a3 = 1 / (2 * t ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * t - (3 * ddq_f - ddq_i) * t ** 2)
        a4 = 1 / (2 * t ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * t + (3 * ddq_f - 2 * ddq_i) * t ** 2)
        a5 = 1 / (2 * t ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * t - (ddq_f - ddq_i) * t ** 2)
        traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
        for i in range(len(traj_pos)):
            traj_pos[i] = round(traj_pos[i], 5)
        traj_pos = np.append([traj_pos],[q_f])
        # print(len(traj_pos))
        return traj_pos

    def Singlep2ptraj(self, pi, pf, t):
        trajectories = [None] * 7
        for i in range(7):
            trajectories[i] = self.fifth_poly(pi[i], pf[i], 0, 0, t)

        trajectories = np.array(trajectories).T
        return trajectories



    # THIS P2P IS DIFFERENT FROM USUAL!!!
    def p2pTraj(self, points):
        tstep = 0.004
        # print(" PLEASE PRINT")
        pointarray = []
        timearray = []
        soundarray = []
        for p in points:
            pointarray.append(p[0])
            timearray.append(p[1])
            soundarray.append(p[2])
            print("POINTARRAY", pointarray)
        if self.sim == False:
            IP = self.getAngle() # IP = self.arm.position
        else:
            IP = [0, 0, 0, 0, 0, 0]
        """print("THREE PRINTS")
        print(IP[1])
        print(pointarray[0])
        print(timearray[0])"""
        traj = self.Singlep2ptraj(IP[1], pointarray[0], timearray[0])

        sound = list(np.linspace(0., 0., math.ceil(timearray[0] / stepT)))
        # self.movexArm(traj, soundarray[0])
        # NEW STUFF

        for x in range(len(traj)):
            start = time.time()
            self.movexArm(traj[x])
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        for i in (range(len(points) - 1)):
            # toAdd =

            traj = self.Singlep2ptraj(pointarray[i], pointarray[i + 1], timearray[i + 1])
            # self.movexArm(traj, soundarray[i + 1])
            # also new stuff
            for x in range(len(traj)):
                start = time.time()
                self.movexArm(traj[x])
                t_elapse = time.time() - start
                while t_elapse < tstep:
                    time.sleep(0.0001)
                    t_elapse = time.time() - start
            # self.movexArm(traj)




            # if soundarray[i+1] > 0:
            #     sound = sound+ list(np.linspace(0., 1., math.ceil(timearray[i+1]/stepT)))
            # else:
            #     sound = sound + list(np.linspace(0., 0., math.ceil(timearray[i+1]/stepT)))
        # return traj,sound
