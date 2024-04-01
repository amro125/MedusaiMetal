import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue

# global vars
time_to_move = 3
counter_threshold = 3

#arm 1
queueOne = Queue()
initPosPoppingOne = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
finPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
currPosOne = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
prevStatusOne = "back"
counterOne = 0
statusOne = False

#arm1 snaking
snaking_tracker = 0
amp = 10
iteration_t = 5
traj_t = np.arange(0, 10, 0.004)

#snaking pos
two_in_p = amp * np.sin((np.pi / iteration_t) * traj_t) + initPosPoppingOne[1]
four_in_p = amp * np.sin((np.pi / iteration_t) * traj_t) + initPosPoppingOne[3]
six_in_p = amp * np.sin((np.pi / iteration_t) * traj_t) + initPosPoppingOne[5]

two_out_p = amp * np.sin((np.pi / iteration_t) * traj_t) + finPosPoppingOne[1]
four_out_p = amp * np.sin((np.pi / iteration_t) * traj_t) + finPosPoppingOne[3]
six_out_p = amp * np.sin((np.pi / iteration_t) * traj_t) + finPosPoppingOne[5]

#snaking vel
two_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
four_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
six_in_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)

two_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
four_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)
six_out_v = amp * (np.pi / iteration_t) * np.cos((np.pi / iteration_t) * traj_t)

queue = [queueOne]
initPosPopping = [initPosPoppingOne]
finPosPopping = [finPosPoppingOne]
currPos = [currPosOne]
currVel = [currVelOne]
currTime = [currTimeOne]
prevStatus = [prevStatusOne]
counter = [counterOne]
status = [statusOne]

no_one_tracker = True

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
        t = t/0
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

def snake(initPos, amp, t, iteration_t):
    traj_t = np.arange(0, t, 0.004)
    two = amp * np.sin((np.pi / iteration_t) * traj_t) + initPos[1]
    four = amp * np.sin((np.pi / iteration_t) * traj_t) + initPos[3]
    six = amp * np.sin((np.pi / iteration_t) * traj_t) + initPos[5]

    for i in range(len(two)):
        currArmAngles = arms[0].angles
        currArmAngles[1] = two[i]
        currArmAngles[3] = four[i]
        currArmAngles[5] = six[i]
        arms[0].set_servo_angle_j(angles=currArmAngles, is_radian=False)

        start_time = time.time()
        tts = time.time() - start_time

        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)


def motifRecognizer(name,*args):
    print("got")
    receivedm = np.array(list(args))
    print(receivedm)
def distance_point_to_line_segment(point, segment_start, segment_end):
    point = np.array(point)
    segment_start = np.array(segment_start)
    segment_end = np.array(segment_end)

    segment_vector = segment_end - segment_start
    point_vector = point - segment_start

    t = np.dot(point_vector, segment_vector) / np.dot(segment_vector, segment_vector)
    t = max(0, min(1, t))
    closest_point = segment_start + t * segment_vector
    distance = np.linalg.norm(point - closest_point)

    return distance
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
            armOneDetect = False
            for i in range(len(rec_arr)):
                x_coord = float(rec_arr[i].split(",")[0])
                if x_coord < 260:
                    armOneDetect = True
                    if prevStatus[0] == "out":
                        counter[0] += 1
                    else:
                        counter[0] = 0
                        prevStatus[0] = "out"
                    if counter[0] > counter_threshold and not(status[0]):
                        status[0] = True
                        print("arm1) out")
                        pose_to_pose(currPos[0], finPosPopping[0], currVel[0], 0, time_to_move-currTime[0], 0)

            if not(armOneDetect):
                if prevStatus[0] == "back":
                    counter[0] += 1
                else:
                    counter[0] = 0
                    prevStatus[0] = "back"
                if counter[0] > counter_threshold and status[0]:
                    status[0] = False
                    print("arm1) go back, no one detected IN RANGE")
                    pose_to_pose(currPos[0], initPosPopping[0], currVel[0], 0, currTime[0], 0)
def robotThread():
    global snaking_tracker
    while True:
        start_time = time.time()
        tts = time.time() - start_time
        if not(queue[0].empty()):
            snaking_tracker = 0
            info = queue[0].get()

            next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1), round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]

            print(str(0) + str(next_pos))
            arms[0].set_servo_angle_j(angles=next_pos, is_radian=False)

            currPos[0] = next_pos
            currTime[0] = info[2]
            currVel[0] = info[4]
        else:
            if (status[0]):
                next_pos = arms[0].angles
                next_pos[1] = two_out_p[snaking_tracker%2500]
                next_pos[3] = four_out_p[snaking_tracker%2500]
                next_pos[5] = six_out_p[snaking_tracker%2500]

                arms[0].set_servo_angle_j(angles = next_pos, is_radian=False)

                currPos[0] = next_pos
                currVel[0] = [0.0, two_out_v[snaking_tracker%2500], 0.0, four_out_v[snaking_tracker%2500], 0.0, six_out_v[snaking_tracker%2500], 0.0]
                snaking_tracker += 1
            else:
                next_pos = arms[0].angles
                next_pos[1] = two_in_p[snaking_tracker%2500]
                next_pos[3] = four_in_p[snaking_tracker%2500]
                next_pos[5] = six_in_p[snaking_tracker%2500]

                arms[0].set_servo_angle_j(angles=next_pos, is_radian=False)

                currPos[0] = next_pos
                currVel[0] = [0.0, two_in_v[snaking_tracker%2500], 0.0, four_in_v[snaking_tracker%2500], 0.0, six_in_v[snaking_tracker%2500], 0.0]
                snaking_tracker += 1

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
    IPstring = [IP0]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.237') # left front
    global arms
    arms = [arm1]

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
    # snake(currPos[0], 10, 100, 10)
    # pose_to_pose(currPos[0], snakeTop, currVel[0], 0, 5, 0)
    # time.sleep(5)
    # pose_to_pose(currPos[0], initPosPopping[0], currVel[0], 0, 5, 0)
    # time.sleep(5)
    # pose_to_pose(currPos[0], snakeBottom, currVel[0], 0, 5, 0)
    # time.sleep(5)
    # pose_to_pose(currPos[0], initPosPopping[0], currVel[0], 0, 5, 0)

    while True:
        input(".")

