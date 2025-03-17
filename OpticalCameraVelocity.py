#!/usr/bin/env python

'''
example to show optical flow

USAGE: opt_flow.py [<video_source>]

Keys:
 1 - toggle HSV flow visualization
 2 - toggle glitch

Keys:
    ESC    - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import math
import cv2
# import video
from collections import deque
import socket
import time



def split_list(input_list):
    array_list = np.array(input_list)
    middle = len(array_list) // 2
    first_half = array_list[:middle].tolist()
    second_half = array_list[middle:].tolist()
    return first_half, second_half


# 400 to 60000

def draw_flow(img, flow, step=16):
    global arrows, arrowsleft, arrowsright, lastPrintTime, velocityArray, allTimeMaxMag, allTimeMaxLocation
    h, w = img.shape[:2]
    mid_w = w // 2
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    maxMag = float("-inf")
    maxLocation = (-1, -1)
    windowlen = 5

    # if len(velocityArray) >= windowlen:
    #     velocityArray.clear()

    count = 0
    for (x1, y1), (x2, y2) in lines:
        # print("X1Y1", count, x1, y1)

        # store magnitude of each point into array, then subract them from each other
        # different things i can use: moving average, record 5 frames and take from first and last
        # idea: use boolean to indicate if acceleration max has been reached in the past 3 seconds
        mag = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
        # print("varray", velocityArray)
        if mag > maxMag:
            maxMag = mag
            maxLocation = (x1, y1)
        arrows.append(mag)
        if x1 < mid_w:
            arrowsleft.append(mag)
        else:
            arrowsright.append(mag)

        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        count += 1
        alltimemax.append(maxMag)
    # print(velocityArray)
    currTime = time.time()
    # if len(velocityArray) == windowlen:
        # print("new mag mag!", maxMag)
    checker = sum(alltimemax) / len(alltimemax)
    allTimeMaxLocation = maxLocation
    # print("ALL TIME", checker)
# Tuned thresholds: All lights on = 25, Near darkness = 20
#Amit Tuned these values on FEB 25
    if currTime - lastPrintTime >= 0.25:
        if checker >= 16: # TODO: lights flashing was 9.9, 11 magnitude
            print("MAX MAGNITUDE OF", checker, "IS AT", allTimeMaxLocation)
            lastPrintTime = currTime
            allTimeMaxMag = -1
            allTimeMaxLocation = (-1, -1)
            return vis, maxLocation, 0

    return vis, None, False


def draw_hsv(flow):
    h, w = flow.shape[:2]
    fx, fy = flow[:, :, 0], flow[:, :, 1]
    ang = np.arctan2(fy, fx) + np.pi
    v = np.sqrt(fx * fx + fy * fy)
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[..., 0] = ang * (180 / np.pi / 2)
    hsv[..., 1] = 255
    hsv[..., 2] = np.minimum(v * 4, 255)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return bgr


def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:, :, 0] += np.arange(w)
    flow[:, :, 1] += np.arange(h)[:, np.newaxis]
    res = cv2.remap(img, flow, None, cv2.INTER_LINEAR)
    return res


def scaleDiff(value, min, max):
    scaled = -(value - min) * (20 - 0) / (max - min) + 0
    return int(scaled)


if __name__ == '__main__':
    import sys
    # to change camera
    # ignore circle region opencv
    y1 = 150 # 200
    y2 = 500
    print(__doc__)
    try:
        fn = sys.argv[1]
    except IndexError:
        fn = 0

    arrows = []
    arrowsleft = []
    arrowsright = []

    cam = cv2.VideoCapture('/dev/video0')
    ret, prev = cam.read()
    prev = prev[y1:y2, :]
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    window = 20
    minMove = 0.01
    maxMove = 700
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    # MESSAGE = "Hello, World!"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    movingaverage = deque(maxlen=window)
    global alltimemax
    alltimemax = deque(maxlen=5)
    lastavg = 0
    lastMove = 0
    lastPrintTime = time.time()
    velocityArray = []
    allTimeMaxMag = float("-inf")
    allTimeMaxLocation = (-1, -1)

    while True:
        ret, frame = cam.read()
        img = frame[y1:y2, :]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(prevgray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        prevgray = gray

        arrows.clear()
        arrowsleft.clear()
        arrowsright.clear()
        finalImg, maxLocation, movementInt = draw_flow(gray, flow)
        if maxLocation is not None:
            if movementInt == 0:
                # send the max location thru socket
                print(np.array(maxLocation), "abrupt movement detected")
                maxLocationBytes = np.array([int(maxLocation[0]),int(maxLocation[1]), 0]).tobytes()
                sock.sendto(maxLocationBytes, (UDP_IP, UDP_PORT))
            else:
                print(np.array(maxLocation), "movement but no abrupt")
                maxLocationBytes = np.array([int(maxLocation[0]), int(maxLocation[1]), movementInt]).tobytes()
                sock.sendto(maxLocationBytes, (UDP_IP, UDP_PORT))
        # output = split_list(arrows)
        # movingaverage.append(np.sum(output[0]))
        movingaverage.append(np.sum(arrows))
        avg = np.sum(movingaverage) / len(movingaverage)
        # print(avg,len(movingaverage))
        scaled = abs(scaleDiff(avg, minMove, maxMove))
        # print(scaled)
        if scaled != lastMove:
            print(scaled, "THIS IS SCALED")
            if np.sum(arrowsright)/len(arrowsright) > np.sum(arrowsleft)/len(arrowsleft) and  np.sum(arrowsright)/len(arrowsright) > 0.5:
                print("right")
                array_to_send = np.array([scaled,0])
            elif np.sum(arrowsleft)/len(arrowsleft) > 0.5:
                print("left")
                array_to_send = np.array([scaled, 2])
            else:
                print("center")
                array_to_send = np.array([scaled, 1])

            # print(array_to_send)
            # Convert the array to bytes
            # array_bytes = array_to_send.tostring()
            # sock.sendto(array_bytes, (UDP_IP, UDP_PORT))
            # sock.sendto(bytes(str(scaled), "utf-8"), (UDP_IP, UDP_PORT))
        lastavg = avg
        lastMove = int(scaled)
        # print(avg)
        cv2.imshow('flow', finalImg)
        if show_hsv:
            cv2.imshow('flow HSV', draw_hsv(flow))
        if show_glitch:
            cur_glitch = warp_flow(cur_glitch, flow)
            cv2.imshow('glitch', cur_glitch)

        ch = cv2.waitKey(5)
        if ch == 27:
            break
        if ch == ord('1'):
            show_hsv = not show_hsv
            print('HSV flow visualization is', ['off', 'on'][show_hsv])
        if ch == ord('2'):
            show_glitch = not show_glitch
            if show_glitch:
                cur_glitch = img.copy()
            print('glitch is', ['off', 'on'][show_glitch])
    cv2.destroyAllWindows()
