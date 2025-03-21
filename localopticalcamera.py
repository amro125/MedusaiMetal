#!/usr/bin/env python

'''
example to show optical flow around detected humans only

USAGE: OpticalCameraVelocityYolo.py [<video_source>]

Keys:
 1 - toggle HSV flow visualization
 2 - toggle glitch
 3 - increase body sensitivity threshold
 4 - decrease body sensitivity threshold
 5 - increase arm sensitivity multiplier
 6 - decrease arm sensitivity multiplier
 7 - increase arm threshold
 8 - decrease arm threshold
 9 - increase edge sensitivity
 0 - decrease edge sensitivity

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

# Global variables
arrows = []
arrowsleft = []
arrowsright = []
alltimemax = deque(maxlen=5)
lastPrintTime = 0
velocityArray = []
allTimeMaxMag = float("-inf")
allTimeMaxLocation = (-1, -1)

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
        x1 = max(0, int(x - w / 2 - padding))
        y1 = max(0, int(y - h / 2 - padding))
        x2 = min(frame.shape[1], int(x + w / 2 + padding))
        y2 = min(frame.shape[0], int(y + h / 2 + padding))
        mask[y1:y2, x1:x2] = 255
    return mask


def get_edge_weight(x, y, img_width, img_height, edge_sensitivity=1.5):
    """Calculate weight based on proximity to frame edge.
    Returns higher values for points near the edge and lower values for points in center.
    """
    # Calculate normalized distance from center (0.0 = center, 1.0 = edge)
    center_x, center_y = img_width / 2, img_height / 2
    dx = abs(x - center_x) / (img_width / 2)
    dy = abs(y - center_y) / (img_height / 2)

    # Use the max of dx and dy as the distance metric (1.0 at corners and edges)
    distance = max(dx, dy)

    # Apply a power function to boost edge weights
    # Higher edge_sensitivity = stronger edge preference
    weight = distance ** edge_sensitivity

    return weight


def draw_flow(img, flow, step=16, body_threshold=70, arm_threshold=40, arm_sensitivity=2.1, edge_sensitivity=1.5):
    global arrows, arrowsleft, arrowsright, lastPrintTime, velocityArray, allTimeMaxMag, allTimeMaxLocation, alltimemax
    h, w = img.shape[:2]
    mid_w = w // 2
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T

    # Calculate flow vectors
    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Add edge weight visualization as a grid overlay
    edge_vis = vis.copy()
    grid_step = 40
    for i in range(0, w, grid_step):
        for j in range(0, h, grid_step):
            edge_weight = get_edge_weight(i, j, w, h, edge_sensitivity)
            # Visualize weight as color intensity (bright = high weight)
            color_intensity = int(255 * edge_weight)
            cv2.circle(edge_vis, (i, j), 3, (0, color_intensity, 0), -1)

    # Blend edge visualization with main visualization
    vis = cv2.addWeighted(vis, 0.7, edge_vis, 0.3, 0)

    # Find average flow direction for global motion detection (walking)
    if len(fx) > 0 and len(fy) > 0:
        avg_fx = np.mean(fx)
        avg_fy = np.mean(fy)
        avg_flow_mag = math.sqrt(avg_fx * avg_fx + avg_fy * avg_fy)
    else:
        avg_fx, avg_fy, avg_flow_mag = 0, 0, 0

    # Draw reference vector for global motion
    cv2.arrowedLine(vis, (30, 120), (30 + int(avg_fx * 3), 120 + int(avg_fy * 3)),
                    (255, 0, 0), 2, tipLength=0.5)
    cv2.putText(vis, "Global Motion", (50, 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    maxMag = float("-inf")
    maxLocation = (-1, -1)
    maxLocalMag = float("-inf")  # For detecting arm movement
    maxLocalLocation = (-1, -1)

    # Store significant local motion vectors (potential arm movements)
    local_motion_vectors = []

    # Color lines based on whether they represent walking or limb movement
    for i, ((x1, y1), (x2, y2)) in enumerate(lines):
        # Calculate magnitude and angle of flow vector
        dx, dy = x2 - x1, y2 - y1
        mag = math.sqrt(dx * dx + dy * dy)

        # Skip tiny movements
        if mag < 0.5:
            continue

        # Calculate how much this vector deviates from global motion
        # Normalize both vectors
        if mag > 0:
            nx, ny = dx / mag, dy / mag
        else:
            nx, ny = 0, 0

        if avg_flow_mag > 0:
            avg_nx, avg_ny = avg_fx / avg_flow_mag, avg_fy / avg_flow_mag
        else:
            avg_nx, avg_ny = 0, 0

        # Dot product to find how aligned this vector is with global motion
        # 1.0 = same direction, -1.0 = opposite, 0 = perpendicular
        alignment = nx * avg_nx + ny * avg_ny

        # Calculate deviation from global motion
        deviation = 1.0 - abs(alignment)  # 0 = aligned with global, 1 = perpendicular

        # Get edge weight for this point's position
        edge_weight = get_edge_weight(x1, y1, w, h, edge_sensitivity)

        # Adjust magnitude based on edge proximity - movements at edges get boosted
        edge_adjusted_mag = mag * edge_weight

        # Determine if this is likely arm/limb movement
        # More strict criteria: higher deviation from global motion and magnitude threshold
        is_limb_movement = deviation > 0.65 and mag > 1.5  # Stricter threshold to avoid misclassifying body as arms

        # Color code the vector based on its type:
        # Red: Global motion (walking) - aligned with average
        # Blue: Local motion (limb) - deviating from average
        # Green: Edge motion - weighted by proximity to edge
        if is_limb_movement:
            color = (255, 0, 0)  # Blue for limb movement
            # Scale magnitude by arm_sensitivity for limb movements
            weighted_mag = edge_adjusted_mag * arm_sensitivity
            local_motion_vectors.append((weighted_mag, (x1, y1)))
        else:
            # Use color gradient based on edge weight
            edge_color = int(255 * edge_weight)
            color = (0, edge_color, 255 - edge_color)  # Blend between red and green based on edge weight
            weighted_mag = edge_adjusted_mag

        # Draw colored flow line with thickness based on edge weight
        line_thickness = max(1, int(2 * edge_weight))
        cv2.line(vis, (x1, y1), (x2, y2), color, line_thickness)

        # Circle size indicates edge importance
        circle_radius = max(1, int(3 * edge_weight))
        cv2.circle(vis, (x1, y1), circle_radius, color, -1)

        # Track maximum magnitudes
        if weighted_mag > maxMag:
            maxMag = weighted_mag
            maxLocation = (x1, y1)

        # Add to tracking arrays
        arrows.append(weighted_mag)
        if x1 < mid_w:
            arrowsleft.append(weighted_mag)
        else:
            arrowsright.append(weighted_mag)

        alltimemax.append(maxMag)

    # Find max local motion (for arm detection)
    if local_motion_vectors:
        local_motion_vectors.sort(reverse=True)
        maxLocalMag = local_motion_vectors[0][0]
        maxLocalLocation = local_motion_vectors[0][1]

        # Highlight the max limb movement point
        if maxLocalLocation != (-1, -1):
            cv2.circle(vis, maxLocalLocation, 5, (255, 0, 0), -1)

    currTime = time.time()

    # Calculate overall coherence of motion vectors (for body vs arm differentiation)
    coherence = 1.0
    if len(arrows) > 5:
        # If vectors are mostly pointing in the same direction, it's body movement
        # If vectors are pointing in different directions, it's more likely arm movement
        directions = []
        for ((x1, y1), (x2, y2)) in lines:
            if x2 != x1 or y2 != y1:  # Avoid division by zero
                angle = math.atan2(y2 - y1, x2 - x1)
                directions.append(angle)

        if directions:
            # Calculate circular standard deviation of angles
            sin_sum = sum(math.sin(a) for a in directions)
            cos_sum = sum(math.cos(a) for a in directions)
            r = math.sqrt(sin_sum ** 2 + cos_sum ** 2) / len(directions)
            # r is close to 1.0 if all vectors point in same direction, close to 0.0 if random
            coherence = r

    # Use either global motion or local motion detection with independent thresholds
    # Now requires more local vectors and checks coherence to avoid false arm detection
    if (len(local_motion_vectors) > 6 and  # Need more vectors to classify as arm movement
            coherence < 0.7 and  # Arm movements are less coherent than body
            maxLocalMag > 0.7 * maxMag):  # Local magnitude should be significant

        checker = maxLocalMag  # Focus on arm movement
        detection_type = "ARM"
        current_threshold = arm_threshold  # Using independent arm threshold
    else:
        checker = sum(alltimemax) / len(alltimemax) if alltimemax else 0
        detection_type = "BODY"
        current_threshold = body_threshold  # Using body threshold

    # Show coherence value for debugging
    cv2.putText(vis, f"Coherence: {coherence:.2f}", (180, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Display values on frame
    cv2.putText(vis, f"Body Threshold: {body_threshold}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(vis, f"Arm Threshold: {arm_threshold}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(vis, f"Edge Sensitivity: {edge_sensitivity:.1f}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(vis, f"Current: {checker:.2f}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(vis, f"Detection: {detection_type}", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    if currTime - lastPrintTime >= 0.25:
        if checker >= current_threshold:
            print(
                f"MAX {detection_type} MAGNITUDE OF {checker:.2f} AT {maxLocalLocation if detection_type == 'ARM' else maxLocation}")
            lastPrintTime = currTime
            allTimeMaxMag = -1
            allTimeMaxLocation = (-1, -1)
            # Return the appropriate location based on detection type
            return vis, maxLocalLocation if detection_type == 'ARM' else maxLocation, 0

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
    y1 = 0  # 200
    y2 = 500
    print(__doc__)
    try:
        fn = sys.argv[1]
    except IndexError:
        fn = 0

    # Settings for sensitivity - now with independent thresholds
    body_threshold = 70  # For walking/whole body movement
    arm_threshold = 40  # For arm/limb movement (independent from body threshold) - increased to avoid false positives
    arm_sensitivity_multiplier = 2.1  # Multiplier to enhance arm movement detection
    edge_sensitivity = 0.1  # Edge weighting factor (higher = more emphasis on edges)

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
    maxMove = 1000

    # Network settings
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    IPToSeu = "192.168.1.2"
    PORTToSeu = 7500
    client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    movingaverage = deque(maxlen=window)
    lastavg = 0
    lastMove = 0
    lastPrintTime = time.time()

    while True:
        ret, frame = cam.read()
        if not ret:
            break

        # Get YOLO detections first (on full frame)
        results = model.track(frame, save=False, persist=True, conf=0.85, verbose=False)
        boxes = results[0].boxes.xywh.cpu()

        # Track human count
        numhuman = len(boxes)
        humans_present = numhuman > 0

        if prevhuman != numhuman:
            client.send_message("/human", numhuman)
            prevhuman = numhuman
            print("human is now", numhuman)

        # Create crop for flow analysis
        img = frame[y1:y2, :]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Skip flow calculation entirely if no humans are present
        if not humans_present:
            flow = np.zeros((gray.shape[0], gray.shape[1], 2), dtype=np.float32)
            # Display a message on the frame
            no_humans_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.putText(no_humans_img, "No humans detected - Motion detection paused",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('flow', no_humans_img)
            prevgray = gray

            # Clear arrays to prevent false detections
            arrows.clear()
            arrowsleft.clear()
            arrowsright.clear()
            alltimemax.clear()

            # Skip the rest of processing if no humans are present
            detection_vis = frame.copy()
            cv2.putText(detection_vis, "No humans detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('Human Detection', detection_vis)
            cv2.imshow('Original', frame)

            # Continue to next frame
            ch = cv2.waitKey(5)
            if ch == 27:
                break
            continue

        # Continue with normal processing when humans are present
        # Create human mask for the cropped region
        # Adjust boxes coordinates to match the cropped frame
        adjusted_boxes = []
        for box in boxes:
            x, y, w, h = box
            # Only include if the person is visible in the cropped region
            if y - h / 2 < y2 and y + h / 2 > y1:
                # Adjust y coordinate for the crop
                adjusted_y = y - y1
                adjusted_boxes.append([x, adjusted_y, w, h])

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
                          (int(x - w / 2), int(y - h / 2)),
                          (int(x + w / 2), int(y + h / 2)),
                          (0, 255, 0), 2)

        cv2.imshow('Human Detection', detection_vis)

        # Also visualize the mask
        mask_vis = cv2.cvtColor(human_mask, cv2.COLOR_GRAY2BGR)
        cv2.imshow('Human Mask', mask_vis)

        prevgray = gray

        arrows.clear()
        arrowsleft.clear()
        arrowsright.clear()

        # Process the flow data with all thresholds
        finalImg, maxLocation, movementInt = draw_flow(
            gray, flow,
            body_threshold=body_threshold,
            arm_threshold=arm_threshold,
            arm_sensitivity=arm_sensitivity_multiplier,
            edge_sensitivity=edge_sensitivity
        )

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
            # Increase body sensitivity threshold (lower value = more sensitive)
            body_threshold = max(5, body_threshold - 2)
            print(f"Body threshold: {body_threshold}")
        if ch == ord('4'):
            # Decrease body sensitivity threshold (higher value = less sensitive)
            body_threshold += 2
            print(f"Body threshold: {body_threshold}")
        if ch == ord('5'):
            # Increase arm sensitivity multiplier
            arm_sensitivity_multiplier += 0.5
            print(f"Arm sensitivity multiplier: {arm_sensitivity_multiplier}")
        if ch == ord('6'):
            # Decrease arm sensitivity multiplier
            arm_sensitivity_multiplier = max(1.0, arm_sensitivity_multiplier - 0.5)
            print(f"Arm sensitivity multiplier: {arm_sensitivity_multiplier}")
        if ch == ord('7'):
            # Increase arm threshold (lower value = more sensitive)
            arm_threshold = max(5, arm_threshold - 2)
            print(f"Arm threshold: {arm_threshold}")
        if ch == ord('8'):
            # Decrease arm threshold (higher value = less sensitive)
            arm_threshold += 2
            print(f"Arm threshold: {arm_threshold}")
        if ch == ord('9'):
            # Increase edge sensitivity
            edge_sensitivity += 0.2
            print(f"Edge sensitivity: {edge_sensitivity:.1f}")
        if ch == ord('0'):
            # Decrease edge sensitivity
            edge_sensitivity = max(0.1, edge_sensitivity - 0.2)
            print(f"Edge sensitivity: {edge_sensitivity:.1f}")

    cv2.destroyAllWindows()
