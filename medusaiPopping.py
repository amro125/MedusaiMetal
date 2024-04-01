import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue
import random

# global vars
time_to_move = 3
counter_threshold = 3
rand_t_low = 4
rand_t_high = 7
rand_amp_low = 7
rand_amp_high = 12

def snaking(rand_t):
    amp_r = random.randint(rand_amp_low, rand_amp_high)
    offset_r = 1
    iteration_t_r = rand_t
    traj_t_r = np.arange(0, (2 * iteration_t_r), 0.004)
    traj_t_r_four = np.arange(offset_r, (2 * iteration_t_r) + offset_r, 0.004)
    print(amp_r)

    two_p_r = amp_r * np.sin((np.pi / iteration_t_r) * traj_t_r)
    four_p_r = amp_r * np.sin((np.pi / iteration_t_r) * traj_t_r)
    six_p_r = amp_r * np.sin((np.pi / iteration_t_r) * traj_t_r)

    # snaking vel
    two_v_r = amp_r * (np.pi / iteration_t_r) * np.cos((np.pi / iteration_t_r) * traj_t_r)
    four_v_r = amp_r * (np.pi / iteration_t_r) * np.cos((np.pi / iteration_t_r) * traj_t_r)
    six_v_r = amp_r * (np.pi / iteration_t_r) * np.cos((np.pi / iteration_t_r) * traj_t_r)
    return [two_p_r, four_p_r, six_p_r, two_v_r, four_v_r, six_v_r]

#arm 1
queueOne = Queue()
initPosPoppingOne = [0.0, -42.0, 5.0, 71.2, -10.1, 58.4, 35.2]
finPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
currPosOne = initPosPoppingOne
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
prevStatusOne = "back"
counterOne = 0
statusOne = False
snaking_tracker_one = 0
rand_t_one = random.randint(rand_t_low, rand_t_high)
mod_one = int((2 * rand_t_one)/.004)
snaking_one = snaking(rand_t_one)

#arm 2
queueTwo = Queue()
initPosPoppingTwo = [0.0, 30.0, 0.0, 90.0, 0.0, 50.0, 0.0]
finPosPoppingTwo = [0.0, 60.0, 0.0, 130.0, 0.0, 0.0, 0.0]
currPosTwo = initPosPoppingTwo
currVelTwo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeTwo = 0
prevStatusTwo = "back"
counterTwo = 0
statusTwo = False
snaking_tracker_two = 0
rand_t_two = random.randint(rand_t_low, rand_t_high)
mod_two = int((2 * rand_t_two)/.004)
snaking_two = snaking(rand_t_two)

#arm 3
queueThree = Queue()
initPosPoppingThree = [109.7, 24.6, 10.0, 74.2, -13.9, 57.0, 0.0]
finPosPoppingThree = [109.7, 65.1, 10.0, 126.5, 13.1, -25.8, 0.0]
currPosThree = initPosPoppingThree
currVelThree = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeThree = 0
prevStatusThree = "back"
counterThree = 0
statusThree = False
snaking_tracker_three = 0
rand_t_three = random.randint(rand_t_low, rand_t_high)
mod_three = int((2 * rand_t_three) / .004)
snaking_three = snaking(rand_t_three)

#arm 4
queueFour = Queue()
initPosPoppingFour = [12.5, -1.2, 0.0, 51.6, 0.0, 68.0, 0.0]
finPosPoppingFour = [12.5, 58.2, 0.0, 107.1, 0.0, -31.5, 0.0]
currPosFour = initPosPoppingFour
currVelFour = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeFour = 0
prevStatusFour = "back"
counterFour = 0
statusFour= False
snaking_tracker_four = 0
rand_t_four = random.randint(rand_t_low, rand_t_high)
mod_four = int((2 * rand_t_four) / .004)
snaking_four = snaking(rand_t_four)

queue = [queueOne, queueTwo, queueThree, queueFour]
initPosPopping = [initPosPoppingOne, initPosPoppingTwo, initPosPoppingThree, initPosPoppingFour]
finPosPopping = [finPosPoppingOne, finPosPoppingTwo, finPosPoppingThree, finPosPoppingFour]
currPos = [currPosOne, currPosTwo, currPosThree, currPosFour]
currVel = [currVelOne, currVelTwo, currVelThree, currVelFour]
currTime = [currTimeOne, currTimeTwo, currTimeThree, currTimeFour]
prevStatus = [prevStatusOne, prevStatusTwo, prevStatusThree, prevStatusFour]
counter = [counterOne, counterTwo, counterThree, counterFour]
status = [statusOne, statusTwo, statusThree, statusFour]
snaking_tracker = [snaking_tracker_one, snaking_tracker_two, snaking_tracker_three, snaking_tracker_four]
snakings = [snaking_one, snaking_two, snaking_three, snaking_four]
mods = [mod_one, mod_two, mod_three, mod_four]

# queue = [queueOne]
# initPosPopping = [initPosPoppingOne]
# finPosPopping = [finPosPoppingOne]
# currPos = [currPosOne]
# currVel = [currVelOne]
# currTime = [currTimeOne]
# prevStatus = [prevStatusOne]
# counter = [counterOne]
# status = [statusOne]
# snaking_tracker = [snaking_tracker_one]

no_one_tracker = True
z1_2_slope = -0.57
z1_2_intercept = 331
overlap = 20
z3_4_slope = 0.8766
z3_4_intercept = -129.51

def zone_1(x, y, x_vel, y_vel):
    line_y = z1_2_slope * x + (z1_2_intercept + overlap)
    dt = time_to_move - currTime[3]
    inzone = x < 250 and y < line_y
    willbeinzone = (x + x_vel * dt) < 250 and (y + y_vel * dt) < line_y
    return inzone or willbeinzone
def zone_2(x, y, x_vel, y_vel):
    line_y = z1_2_slope * x + (z1_2_intercept - overlap)
    dt = time_to_move - currTime[0]
    inzone = x < 300 and y > line_y
    willbeinzone = (x + x_vel * dt) < 300 and (y + y_vel * dt) > line_y
    return inzone or willbeinzone

def zone_3(x, y, x_vel, y_vel):
    dt = time_to_move-currTime[2]
    line_y = z3_4_slope * x + (z3_4_intercept - overlap)
    inzone = 260 <= x and y >= line_y
    willbeinzone = 260 <= (x + x_vel * dt) and (y + y_vel * dt) > line_y
    return inzone or willbeinzone
def zone_4(x, y, x_vel, y_vel):
    dt = time_to_move - currTime[1]
    line_y = z3_4_slope * x + (z3_4_intercept - overlap)
    inzone = x >= 400 and y <= line_y
    willbeinzone = (x + x_vel * dt) >= 400 and (y + y_vel * dt) <= line_y
    return inzone or willbeinzone

def strumbot(traj):
    # j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)

def fifth_poly(q_i, q_f, v_i, vf, t):
    if (t == 0):
        print("STOP")
        print(q_i)
        print(q_f)
        print(v_i)
        print(vf)
    # time/0.005
    traj_t = np.arange(0, t, 0.004)
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
    traj_vel = a1  + 2*a2 * traj_t ** 1 + 3*a3 * traj_t ** 2 + 4*a4 * traj_t ** 3 + 5*a5 * traj_t ** 4
    return traj_pos, traj_vel

def setup(strumD):
    i = 0
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring[i].copy()
        i += 1
        # angle[0] = angle[0]- strumD/2
        print(angle)
        a.set_servo_angle(angle=angle, wait=True, speed=10, acceleration=0.25, is_radian=False)

def pose_to_pose(initPos, finPos, vi, vf, t, armNum):
    print(armNum)
    global queue
    with queue[armNum].mutex:
        queue[armNum].queue.clear()
        queue[armNum].all_tasks_done.notify_all()
        queue[armNum].unfinished_tasks = 0

    currArmAngles = [round(arms[armNum].angles[0], 1), round(arms[armNum].angles[1], 1), round(arms[armNum].angles[2], 1),
                     round(arms[armNum].angles[3], 1), round(arms[armNum].angles[4], 1), round(arms[armNum].angles[5], 1),
                     round(arms[armNum].angles[6], 1)]

    if t == 0:
        print("T IS ZERO")
        print(arms[armNum].angles)
        print(finPos)
        # t = t/0
        # arms[armNum].set_servo_angle(angle=finPos, wait=True, speed=10, acceleration=0.25, is_radian=False)
        # currPos[armNum] = finPos
    else:
        posTrajs = []
        velTrajs = []
        for i in range(len(initPos)):  # generate trajs and vels for each joint
            tempTraj, tempVel = fifth_poly(initPos[i], finPos[i], vi[i], vf, t)  # tempTraj, tempVel has t/.004 length
            posTrajs.append(tempTraj)
            velTrajs.append(tempVel)
        reformatPos = [currArmAngles]
        reformatVel = [currVel[armNum]]
        for i in range(len(posTrajs[0])):
            posRow = []
            velRow = []
            for j in range(7):
                posRow.append(posTrajs[j][i])
                velRow.append(velTrajs[j][i])
            if finPos == finPosPopping[armNum]:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] + ((i+1) * 0.004), reformatVel[-1],
                          velRow])  # each queue item has curr_pos, next_pos, next_time, curr_vel, next_vel for all joints
            else:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] - ((i + 1) * 0.004), reformatVel[-1],
                                   velRow])
            reformatPos.append(posRow)
            reformatVel.append(velRow)

def visionThread():
    global no_one_tracker
    global arms
    global counter
    global prevStatus
    global status

    while True:
        data, addr = sock.recvfrom(1024)
        # print(data.decode("utf-8"))
        if ("noone" in data.decode("utf-8")):
            for i in range(len(arms)):
                if prevStatus[i] == "back":
                    counter[i] += 1
                else:
                    counter[i] = 0
                    prevStatus[i] = "back"
                if counter[i] > 5 and status[i]:
                    status[i] = False
                    print("arm" + str(i) + ") go back, no one detected")
                    pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)
        else:
            rec_arr = ast.literal_eval(data.decode("utf-8"))
            armDetect = [False, False, False, False]
            # print(data.decode("utf-8"))
            # print(rec_arr)
            for i in range(len(rec_arr)):
                coordinates = rec_arr[i].split("/")[0].split(",")
                vel = rec_arr[i].split("/")[1].split(",")
                x_coord = float(coordinates[0])
                y_coord = float(coordinates[1])
                x_vel = float(vel[0])
                y_vel = float(vel[1])
                if zone_3(x_coord, y_coord, x_vel, y_vel):
                    armDetect[2] = True
                    if prevStatus[2] == "out":
                        counter[2] += 1
                    else:
                        counter[2] = 0
                        prevStatus[2] = "out"
                    if counter[2] > counter_threshold and not(status[2]):
                        status[2] = True
                        print("arm3) out")
                        pose_to_pose(currPos[2], finPosPopping[2], currVel[2], 0, time_to_move-currTime[2], 2)
                if zone_2(x_coord, y_coord, x_vel, y_vel):
                    armDetect[0] = True
                    if prevStatus[0] == "out":
                        counter[0] += 1
                    else:
                        counter[0] = 0
                        prevStatus[0] = "out"
                    if counter[0] > counter_threshold and not(status[0]):
                        status[0] = True
                        print("arm1) out")
                        pose_to_pose(currPos[0], finPosPopping[0], currVel[0], 0, time_to_move-currTime[0], 0)
                if zone_1(x_coord, y_coord, x_vel, y_vel):
                    armDetect[3] = True
                    if prevStatus[3] == "out":
                        counter[3] += 1
                    else:
                        counter[3] = 0
                        prevStatus[3] = "out"
                    if counter[3] > counter_threshold and not(status[3]):
                        status[3] = True
                        print("arm4) out")
                        pose_to_pose(currPos[3], finPosPopping[3], currVel[3], 0, time_to_move - currTime[3], 3)
                if zone_4(x_coord, y_coord, x_vel, y_vel):
                    armDetect[1] = True
                    if prevStatus[1] == "out":
                        counter[1] += 1
                    else:
                        counter[1] = 0
                        prevStatus[1] = "out"
                    if counter[1] > counter_threshold and not(status[1]):
                        status[1] = True
                        print("arm2) out")
                        pose_to_pose(currPos[1], finPosPopping[1], currVel[1], 0, time_to_move-currTime[1], 1)

            for i in range(len(armDetect)):
                if not(armDetect[i]):
                    if prevStatus[i] == "back":
                        counter[i] += 1
                    else:
                        counter[i] = 0
                        prevStatus[i] = "back"
                    if counter[i] > counter_threshold and status[i]:
                        status[i] = False
                        print("arm"+str(i)+") go back, no one detected IN RANGE")
                        pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)
def robotThread():
    global snaking_tracker_one
    global snaking_tracker_two
    global snaking_tracker_three
    global snaking_tracker_four

    while True:
        start_time = time.time()
        tts = time.time() - start_time
        for i in range(len(arms)):
            if not(queue[i].empty()):
                snaking_tracker[i] = 0
                info = queue[i].get()
                next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1),
                            round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

                arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
                currPos[i] = next_pos
                currTime[i] = info[2]
                currVel[i] = info[4]
            else:
                if (status[i]):
                    next_pos = arms[i].angles
                    next_pos[1] = snakings[i][0][snaking_tracker[i] % mods[i]] + finPosPopping[i][1]
                    next_pos[3] = snakings[i][1][snaking_tracker[i] % mods[i]] + finPosPopping[i][3]
                    next_pos[5] = snakings[i][2][snaking_tracker[i] % mods[i]] + finPosPopping[i][5]
                    currVel[i] = [0.0, snakings[i][3][snaking_tracker[i] % mods[i]], 0.0,
                                  snakings[i][4][snaking_tracker[i] % mods[i]], 0.0, snakings[i][5][snaking_tracker[i] % mods[i]],
                                  0.0]
                else:
                    next_pos = arms[i].angles
                    next_pos[1] = snakings[i][0][snaking_tracker[i] % mods[i]] + initPosPopping[i][1]
                    next_pos[3] = snakings[i][1][snaking_tracker[i] % mods[i]] + initPosPopping[i][3]
                    next_pos[5] = snakings[i][2][snaking_tracker[i] % mods[i]] + initPosPopping[i][5]
                    currVel[i] = [0.0, snakings[i][3][snaking_tracker[i] % mods[i]], 0.0, snakings[i][4][snaking_tracker[i] % mods[i]],
                                  0.0, snakings[i][5][snaking_tracker[i] % mods[i]], 0.0]

                arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
                currPos[i] = next_pos
                snaking_tracker[i] += 1

        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)


if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring

    IP0 = initPosPoppingOne
    IP1 = initPosPoppingTwo
    IP2 = initPosPoppingThree
    IP3 = initPosPoppingFour
    IPstring = [IP0, IP1, IP2, IP3]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5006
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.237') # middle left
    arm2 = XArmAPI('192.168.1.204') # far right
    arm3 = XArmAPI('192.168.1.244') # middle right
    arm4 = XArmAPI('192.168.1.236')  # far left
    global arms
    arms = [arm1, arm2, arm3, arm4]
    # arms = [arm1]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    visionThread = Thread(target=visionThread)
    robotThread = Thread(target=robotThread)
    visionThread.start()
    robotThread.start()

    while True:
        input(".")



    #we want interrupt to happen at time x
    #get the vel at time x would be trajvel[x/0.004] = vinterrupt
    #get the pos at time x would be trajpos[x/0.004] = qinterrupt
    #make new trajectory fifth_poly(qinterrupt, q_f, vinterrupt, vf still 0, time from when you interrupt to new position):
    # vision thread puts distance as q.put('distance')
    #robot thread reads distance as dist = q.get()
