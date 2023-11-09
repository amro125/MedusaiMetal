import atexit
import os
import sys
import time
import numpy as np
import math
from xarm.wrapper import XArmAPI
from pythonosc import udp_client
from pythonosc import dispatcher
from pythonosc import osc_server
import socket
import threading

def strumbot(traj):
    # j_angles = pos
    for i in range(len(traj)):
        # run command
        start_time = time.time()
        j_angles = traj[i]
        arms[0].set_servo_angle_j(angles=j_angles, is_radian=False)
        tts = time.time() - start_time
        # print(j_angles)
        while tts < 0.004:
            # print(tts)
            tts = time.time() - start_time
            time.sleep(0.0001)

def fifth_poly(q_i, q_f, v_i, vf, t):
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
    return traj_pos

def setup(strumD):
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring.copy()
        # angle[0] = angle[0]- strumD/2
        a.set_servo_angle(angle=angle, wait=True, speed=10, acceleration=0.25, is_radian=False)

def pose_to_pose(initPos, finPos, vi, vf, t):
    print(arms[0].angles)
    global moving
    currArmAngles = [round(arms[0].angles[0], 1), round(arms[0].angles[1], 1), round(arms[0].angles[2], 1), round(arms[0].angles[3], 1), round(arms[0].angles[4], 1), round(arms[0].angles[5], 1), round(arms[0].angles[6], 1)]
    if (initPos == currArmAngles and moving == False):
        moving = True
        trajs = []
        for i in range(len(initPos)):
            tempTraj = fifth_poly(initPos[i], finPos[i], vi, vf, t)
            trajs.append(tempTraj)
        reformat = []
        for i in range(len(trajs[0])):
            row = []
            for j in range(7):
                row.append(trajs[j][i])
            reformat.append(row)
        strumbot(reformat)
        moving = False
    else:
        print("init pos not the same")

def motifRecognizer(name,*args):
    print("got")
    receivedm = np.array(list(args))
    print(receivedm)

def distance_point_to_line_segment(point, segment_start, segment_end):
    # Convert the inputs to NumPy arrays for vector operations
    point = np.array(point)
    segment_start = np.array(segment_start)
    segment_end = np.array(segment_end)

    # Calculate the vector representing the line segment
    segment_vector = segment_end - segment_start

    # Calculate the vector from the start of the line segment to the point
    point_vector = point - segment_start

    # Calculate the projection of the point vector onto the line segment vector
    t = np.dot(point_vector, segment_vector) / np.dot(segment_vector, segment_vector)

    # Ensure t is within the bounds of the line segment
    t = max(0, min(1, t))

    # Calculate the closest point on the line segment
    closest_point = segment_start + t * segment_vector

    # Calculate the distance from the point to the closest point on the line segment
    distance = np.linalg.norm(point - closest_point)

    return distance

if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring
    IPstring = [0.0, -30.0, 0.0, 125.0, 0.0, 11.0, -45.0]

    UDP_IP="0.0.0.0"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    arm1 = XArmAPI('192.168.1.236')
    global arms
    arms = [arm1]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    initPosPopping = [0.0, -28.0, 5.0, 97.1, -10.1, -0.9, 35.2]
    finPosPopping = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]

    global moving
    moving = False

    global detected
    detected = False

    pose_to_pose(IPstring, initPosPopping, 0, 0, 5)

    line_start = [227, 219]
    line_end = [39, 359]

    while moving is False:
        data, addr = sock.recvfrom(1024)
        if ("noone" in data.decode("utf-8")):
            if detected:
                pose_to_pose(initPosPopping, finPosPopping, 0, 0, 5)
                print("go back, no one detected")
            detected = False
        else:
            tempDetected = False
            res = data.decode("utf-8").split(":")[1].split(",")
            numppl = int(data.decode("utf-8").split(":")[0])
            for i in range(numppl):
                coords = [float(res[0].strip()), float(res[1].strip())]
                dist = distance_point_to_line_segment(coords, line_start, line_end)
                if dist < 30:
                    tempDetected = True
            if tempDetected and detected is False:
                detected = True
                pose_to_pose(initPosPopping, finPosPopping, 0, 0, 5)
                print("move out")
            elif tempDetected and detected is True:
                print("remain out")
            elif tempDetected is False and detected is True:
                pose_to_pose(finPosPopping, initPosPopping, 0, 0, 5)
                print("go back")
            else:
                print("remain in")

        # while True:
        # pose_to_pose(initPosPopping, finPosPopping, 0, 0, 5)
        # pose_to_pose(finPosPopping, initPosPopping, 0, 0, 5)


    # initial pos: [0, -28, 5, 97.1, -10.1, -0.9, 35.2]
    # final pos: [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
