#!/usr/bin/env python

'''
example to show optical flow around detected humans only

USAGE: OpticalCameraVelocityYolo.py [<video_source>]

Keys:
 1 - toggle HSV flow visualization
 2 - toggle glitch
 3 - increase sensitivity threshold
 4 - decrease sensitivity threshold
 
 ESC - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import math
import cv2
from collections import deque
import socket
import time
from ultralytics import YOLO
from pythonosc import udp_client

model = YOLO('yolov8n.pt')

def split_list(input_list):
    array_list = np.array(input_list)
    middle = len(array_list) // 2
    first_half = array_list[:middle].tolist()
    second_half = array_list[middle:].tolist()
    return first_half, second_half

def create_human_mask(frame, boxes, padding=20):
    """Create a binary mask highlighting only areas around humans with padding"""
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for box in boxes:
        x, y, w, h = box
        x1 = max(0, int(x - w/2 - padding))
        y1 = max(0, int(y - h/2 - padding))
        x2 = min(frame.shape[1], int(x + w/2 + padding))
        y2 = min(frame.shape[0], int(y + h/2 + padding))
        mask[y1:y2, x1:x2] = 255
    return mask

def draw_flow(img, flow, step=16, sensitivity_threshold=31):
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

    count = 0
    for (x1, y1), (x2, y2) in lines:
        # Calculate magnitude of flow vector
        mag = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
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
    
    currTime = time.time()
    checker = sum(alltimemax) / len(alltimemax)
    allTimeMaxLocation = maxLocation
    
    # Display current threshold on frame
    cv2.putText(vis, f"Threshold: {sensitivity_threshold}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(vis, f"Current: {checker:.2f}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    if currTime - lastPrintTime >= 0.25:
        if checker >= sensitivity_threshold:
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
    prevhuman = 0
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
    prev_mask = np.ones_like(prevgray) * 255  # Initial mask (full frame)
    
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    window = 20
    minMove = 0.01
    maxMove = 700
    
    # Network settings
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    IPToSeu = "192.168.1.2"
    PORTToSeu = 7500
    client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    # Variable sensitivity threshold
    sensitivity_threshold = 31
    
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
        if not ret:
            break
            
        # Get YOLO detections first (on full frame)
        results = model.track(frame, save=False, persist=True, conf=0.85, verbose=False)
        boxes = results[0].boxes.xywh.cpu()
        
        # Track human count
        numhuman = len(boxes)
        if prevhuman != numhuman:
            client.send_message("/human", numhuman)
            prevhuman = numhuman
            print("human is now", numhuman)
        
        # Create crop for flow analysis
        img = frame[y1:y2, :]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Create human mask for the cropped region
        if len(boxes) > 0:
            # Adjust boxes coordinates to match the cropped frame
            adjusted_boxes = []
            for box in boxes:
                x, y, w, h = box
                # Only include if the person is visible in the cropped region
                if y - h/2 < y2 and y + h/2 > y1:
                    # Adjust y coordinate for the crop
                    adjusted_y = y - y1
                    adjusted_boxes.append([x, adjusted_y, w, h])
            
            if adjusted_boxes:
                # Create mask for the cropped region
                human_mask = create_human_mask(img, adjusted_boxes)
                
                # Apply mask to gray images for flow calculation
                masked_gray = cv2.bitwise_and(gray, gray, mask=human_mask)
                masked_prevgray = cv2.bitwise_and(prevgray, prevgray, mask=human_mask)
                
                # Calculate optical flow only in masked regions
                flow = cv2.calcOpticalFlowFarneback(masked_prevgray, masked_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                
                # Visualize human detection areas
                detection_vis = frame.copy()
                for box in boxes:
                    x, y, w, h = map(int, box)
                    cv2.rectangle(detection_vis, 
                                 (int(x - w/2), int(y - h/2)), 
                                 (int(x + w/2), int(y + h/2)), 
                                 (0, 255, 0), 2)
                
                cv2.imshow('Human Detection', detection_vis)
                
                # Also visualize the mask
                mask_vis = cv2.cvtColor(human_mask, cv2.COLOR_GRAY2BGR)
                cv2.imshow('Human Mask', mask_vis)
            else:
                # No humans in the cropped area
                flow = np.zeros((gray.shape[0], gray.shape[1], 2), dtype=np.float32)
        else:
            # No humans detected at all
            flow = np.zeros((gray.shape[0], gray.shape[1], 2), dtype=np.float32)
        
        prevgray = gray
        
        arrows.clear()
        arrowsleft.clear()
        arrowsright.clear()
        
        # Process the flow data with current sensitivity threshold
        finalImg, maxLocation, movementInt = draw_flow(gray, flow, sensitivity_threshold=sensitivity_threshold)
        
        # Handle movement detection
        if maxLocation is not None:
            if movementInt == 0:
                # send the max location thru socket
                maxLocationBytes = np.array([int(maxLocation[0]), int(maxLocation[1]), 0]).tobytes()
                sock.sendto(maxLocationBytes, (UDP_IP, UDP_PORT))
            else:
                maxLocationBytes = np.array([int(maxLocation[0]), int(maxLocation[1]), movementInt]).tobytes()
                sock.sendto(maxLocationBytes, (UDP_IP, UDP_PORT))
        
        cv2.imshow('flow', finalImg)
        cv2.imshow('Original', frame)
        
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
        if ch == ord('3'):
            # Increase sensitivity threshold (lower value = more sensitive)
            sensitivity_threshold = max(5, sensitivity_threshold - 2)
            print(f"Sensitivity threshold: {sensitivity_threshold}")
        if ch == ord('4'):
            # Decrease sensitivity threshold (higher value = less sensitive)
            sensitivity_threshold += 2
            print(f"Sensitivity threshold: {sensitivity_threshold}")
            
    cv2.destroyAllWindows()
