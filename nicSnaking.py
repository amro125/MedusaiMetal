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
rand_t_high = 6
rand_amp_low = 7
rand_amp_high = 11

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
initPosPoppingOne = [11.4, -42.6, 5.0, 52.2, -10.1, 49.9, 35.2]
currPosOne = initPosPoppingOne
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
snaking_tracker_one = 0
rand_t_one = random.randint(rand_t_low, rand_t_high)
mod_one = int((2 * rand_t_one)/.004)
snaking_one = snaking(rand_t_one)

#arm 2
queueTwo = Queue()
initPosPoppingTwo = [46.4, 54.6, 0.0, 126.3, 0.0, 26.6, 0.0]
currPosTwo = initPosPoppingTwo
currVelTwo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeTwo = 0
snaking_tracker_two = 0
rand_t_two = random.randint(rand_t_low, rand_t_high)
mod_two = int((2 * rand_t_two)/.004)
snaking_two = snaking(rand_t_two)

#arm 3
queueThree = Queue()
initPosPoppingThree = [166.0, 26.0, 1.3, 100.2, 19.1, 6.5, 0.0]
currPosThree = initPosPoppingThree
currVelThree = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeThree = 0
snaking_tracker_three = 0
rand_t_three = random.randint(rand_t_low, rand_t_high)
mod_three = int((2 * rand_t_three) / .004)
snaking_three = snaking(rand_t_three)

#arm 4
queueFour = Queue()
initPosPoppingFour = [-2.8, 17.1, 0.0, 80.5, 0.0, 59.5, 0.0]
currPosFour = initPosPoppingFour
currVelFour = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeFour = 0
snaking_tracker_four = 0
rand_t_four = random.randint(rand_t_low, rand_t_high)
mod_four = int((2 * rand_t_four) / .004)
snaking_four = snaking(rand_t_four)

#arm 5
queueFive = Queue()
initPosPoppingFive = [180.0, -43.0, 0.0, 74.8, 0.0, 85.4, 0.0]
currPosFive = initPosPoppingFive
currVelFive = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeFive = 0
snaking_tracker_five = 0
rand_t_five = random.randint(rand_t_low, rand_t_high)
mod_five = int((2 * rand_t_five) / .004)
snaking_five = snaking(rand_t_five)

#arm 6
queueSix = Queue()
initPosPoppingSix = [-8.5, -54.9, 12.0, 103.5, -23.3, 79.5, 18.3]
currPosSix = initPosPoppingSix
currVelSix = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeSix = 0
snaking_tracker_six = 0
rand_t_six = random.randint(rand_t_low, rand_t_high)
mod_six = int((2 * rand_t_six) / .004)
snaking_six = snaking(rand_t_six)

#arm 7
queueSeven = Queue()
initPosPoppingSeven = [0.0, -30.0, 25.0, 120.0, 42.0, -14.0, 0.0]
currPosSeven = initPosPoppingSeven
currVelSeven = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeSeven = 0
snaking_tracker_seven = 0
rand_t_seven = random.randint(rand_t_low, rand_t_high)
mod_seven = int((2 * rand_t_seven) / 0.004)
snaking_seven = snaking(rand_t_seven)

queue = [queueOne, queueTwo, queueThree, queueFour, queueFive, queueSix, queueSeven]
initPosPopping = [initPosPoppingOne, initPosPoppingTwo, initPosPoppingThree, initPosPoppingFour, initPosPoppingFive, initPosPoppingSix, initPosPoppingSeven]
currPos = [currPosOne, currPosTwo, currPosThree, currPosFour, currPosFive, currPosSix, currPosSeven]
currVel = [currVelOne, currVelTwo, currVelThree, currVelFour, currVelFive, currVelSix, currVelSeven]
currTime = [currTimeOne, currTimeTwo, currTimeThree, currTimeFour, currTimeFive, currTimeSix, currTimeSeven]
snaking_tracker = [snaking_tracker_one, snaking_tracker_two, snaking_tracker_three, snaking_tracker_four, snaking_tracker_five, snaking_tracker_six, snaking_tracker_seven]
snakings = [snaking_one, snaking_two, snaking_three, snaking_four, snaking_five, snaking_six, snaking_seven]
mods = [mod_one, mod_two, mod_three, mod_four, mod_five, mod_six, mod_seven]

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

def robotThread():
    global snaking_tracker_one
    global snaking_tracker_two
    global snaking_tracker_three
    global snaking_tracker_four
    global snaking_tracker_five
    global snaking_tracker_six
    global snaking_tracker_seven

    while True:
        start_time = time.time()
        tts = time.time() - start_time
        for i in range(len(arms)):
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
    IP4 = initPosPoppingFive
    IP5 = initPosPoppingSix
    IP6 = initPosPoppingSeven
    IPstring = [IP0, IP1, IP2, IP3, IP4, IP5, IP6]

    arm1 = XArmAPI('192.168.1.237') # middle left
    arm2 = XArmAPI('192.168.1.204') # far right
    arm3 = XArmAPI('192.168.1.244') # middle right
    arm4 = XArmAPI('192.168.1.236')  # far left
    arm5 = XArmAPI('192.168.1.208')  # far back left
    arm6 = XArmAPI('192.168.1.234') # far back right
    arm7 = XArmAPI('192.168.1.242')
    global arms
    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7]
    # arms = [arm1]

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        a.set_mode(1)
        a.set_state(0)
    time.sleep(0.5)

    robotThread = Thread(target=robotThread)
    robotThread.start()

    while True:
        input(".")


