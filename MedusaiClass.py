import numpy as np
import math
headS = 2

class Robots:
    def __init__(self, IPadress,startPos, startT, phase = 0):
        self.arm = IPadress
        self.initPos = startPos
        self.phase = phase
        self.strum1 = []
        self.strum2 = []
        self.startT = startT
        self.endT = startT + sum(startPos[0][1])

    
    def test(self,te):
        print(self.arm+self.initPos, te)
        return (self.arm+self.initPos)
    
    def test2(self):
        print(self.test(3)*10)
        self.strum8 = [4, 5]

    
 
    def dancepos(self, t):
        amp5 = headS * 8.6
        amp3 = 0.4 * amp5
        amp1 = 0.2 * amp5
        amp5extra = 30
        amptwist = 200
        headS5 = 0.045 * amptwist * headS
        headS3 = 0.09 * amptwist * headS

        j_angles = self.initPos.copy()
        strike = (math.sin((math.pi / headS3) * (t)))
        two = math.sin((math.pi / headS) * (t - 0.75*headS-self.phase*headS))
        three = math.sin((math.pi / headS3) * (t-self.phase*headS3))
        four = math.sin((math.pi / headS) * (t-0.5*headS-self.phase*headS))
        five = math.sin((math.pi / headS5) * (t-self.phase*headS5))
        six = math.sin((math.pi / (headS) * (t-self.phase*headS)))

        # j_angles[0] = startp[0] - baseamp * strike
        j_angles[1] = self.initPos[1] - amp1 * two
        j_angles[2] = self.initPos[2] + amptwist * three
        j_angles[3] = self.initPos[3] + amp3 * four
        j_angles[4] = self.initPos[4] + amptwist * five
        j_angles[5] = self.initPos[5] - amp5 * six
        j_angles[6] = self.initPos[6] - (amptwist * five + amptwist * three - baseamp * strike)
        if abs(j_angles[6]) >= 360:
            j_angles[6] = (j_angles[6]/abs(j_angles[6]))*359.8
        return j_angles

    def makeSnakeTraj(self):
        snakeTraj = []
        timeArray = np.arange(self.tStart,self.tEnd,0.004)
        for time in timeArray:
            snakeTraj.append(self.dancepos(time))
        return snakeTraj

    def matlabparser(self, traj, v_i, v_f, IP):
    # format is [[[pos1,pos2],[time1,time2]],[[JOINT2pos1,pos2],[time1,time2]]]
        trajblock = []
        j = 0
        for joint in traj:
            # format is now at [[pos1,pos2],[time1,time2]]
            jointblock = np.empty([0, 1])
            if len(joint[0]) == 1:
                iter = 1
            else:
                iter = len(joint[0])
            for i in range(iter):
                if i == 0:
                    v = v_i[j]
                    vf = 0
                    qi = IP[j]
                    qf = joint[0][i]
                elif i == iter - 1:
                    v = 0
                    vf = v_f[j]
                    qi = joint[0][i - 1]
                    qf = joint[0][i]
                else:
                    v = 0
                    vf = 0
                    qi = joint[0][i - 1]
                    qf = joint[0][i]
                timescale = strumtime * joint[1][i] / sum(joint[1])
                step = fifth_poly(qi, qf, v, vf, timescale)
                jointblock = np.append(jointblock, step)
            # print("JOINT BLOCK",j,len(jointblock))
            trajblock.append(jointblock)
            j += 1
        # print(len(trajblock[3]))
        # reformat
        reformat = []
        for i in range(len(trajblock[0])):
            row = []
            # for q in range(7):
            # print(len(trajblock[q]))
            for j in range(7):
                row.append(trajblock[j][i])
            reformat.append(row)
        return reformat

    def makeStrum(self,struminput):
        tEnd = self.startT + sum(struminput[i][1])
        endpos = self.dancepos(self.initPos, self.endT, self.phase)
        for i in range(7):
            struminput[i][0][-1] = endpos[i]
        vstruminit = self.dancevel(self.startT)
        vfinal = self.dancevel(tEnd)
        play = self.matlabparser(struminput,vstruminit,vfinal, armIP)
        noplay = []
        tnoplay = np.arange(self.startT, tEnd, 0.004)
        for time in tnoplay:
            noplay.append(dancepos(armIP,time,phase))
        trajectory = [play,noplay]
        return trajectory


r1 = Robots(2,7,4,1,2)
r1.test2()
print(r1.startT)
print(r1.strum8)
