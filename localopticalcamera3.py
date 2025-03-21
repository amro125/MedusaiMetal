#!/usr/bin/env python

'''
Script to detect human movement using only YOLO detection (no optical flow)

USAGE: YoloVelocityTracker.py [<video_source>]

Keys:
 1 - increase body movement threshold
 2 - decrease body movement threshold
 3 - increase limb movement threshold
 4 - decrease limb movement threshold
 5 - toggle debug visualization

 ESC - exit
'''

import numpy as np
import cv2
import socket
import time
import sys
from collections import deque
from ultralytics import YOLO
from pythonosc import udp_client

# Global variables
prevhuman = 0
body_threshold = 15  # Threshold for overall body movement (pixels)
limb_threshold = 35  # Threshold for limb movement (pixels)
show_debug = True
center_pixels_threshold = 70  # How many pixels from center is considered "at center"
center_time_threshold = 20.0  # Time in seconds to be at center for "pluck" message
inactivity_threshold = 120.0  # Time in seconds with no movement for "reward" message

# History of detections for tracking
detection_history = {}  # Dictionary to store previous detections by track_id
previous_boxes = []  # List to store previous frame boxes for non-tracked mode

# Create a deque to store maximum velocities for smoothing
max_velocity_history = deque(maxlen=5)
max_limb_velocity_history = deque(maxlen=5)

# Initialize YOLO model
model = YOLO('yolov8n.pt')


def calculate_velocities(current_boxes, previous_boxes):
    """
    Calculate body and limb movement velocities by comparing current and previous detections
    Returns:
        body_velocities: List of (velocity, position) tuples for body movement
        limb_velocities: List of (velocity, position) tuples for limb movement
    """
    body_velocities = []
    limb_velocities = []

    # If we don't have previous detections, return empty lists
    if len(previous_boxes) == 0 or len(current_boxes) == 0:
        return body_velocities, limb_velocities

    # Try to match current boxes with previous boxes
    # This is a simple matching based on IoU (Intersection over Union)
    for curr_box in current_boxes:
        curr_x, curr_y, curr_w, curr_h = curr_box
        curr_center = (curr_x, curr_y)

        best_match_idx = -1
        best_match_dist = float('inf')

        # Find closest previous box
        for i, prev_box in enumerate(previous_boxes):
            prev_x, prev_y, prev_w, prev_h = prev_box
            prev_center = (prev_x, prev_y)

            # Calculate center distance
            center_dist = np.sqrt((curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2)

            if center_dist < best_match_dist:
                best_match_dist = center_dist
                best_match_idx = i

        # If we found a reasonably close match
        if best_match_idx >= 0 and best_match_dist < 200:  # Max distance threshold
            prev_x, prev_y, prev_w, prev_h = previous_boxes[best_match_idx]
            prev_center = (prev_x, prev_y)

            # Calculate body velocity (center movement)
            body_velocity = best_match_dist
            body_velocities.append((body_velocity, curr_center))

            # Calculate limb velocity (change in width/height)
            width_change = abs(curr_w - prev_w)
            height_change = abs(curr_h - prev_h)
            limb_velocity = max(width_change, height_change)
            limb_velocities.append((limb_velocity, curr_center))

    return body_velocities, limb_velocities


def find_max_movement(body_velocities, limb_velocities, body_threshold, limb_threshold):
    """
    Find the maximum movement and determine if it's a body or limb movement
    """
    max_body_velocity = 0
    max_body_location = (-1, -1)
    max_limb_velocity = 0
    max_limb_location = (-1, -1)
    movement_type = None
    movement_location = None

    # Find max body velocity
    if body_velocities:
        max_body_velocity, max_body_location = max(body_velocities, key=lambda x: x[0])
        max_velocity_history.append(max_body_velocity)

    # Find max limb velocity
    if limb_velocities:
        max_limb_velocity, max_limb_location = max(limb_velocities, key=lambda x: x[0])
        max_limb_velocity_history.append(max_limb_velocity)

    # Get smoothed values
    smoothed_body_velocity = sum(max_velocity_history) / len(max_velocity_history) if max_velocity_history else 0
    smoothed_limb_velocity = sum(max_limb_velocity_history) / len(
        max_limb_velocity_history) if max_limb_velocity_history else 0

    # Determine if movement exceeds threshold
    if smoothed_body_velocity > body_threshold:
        movement_type = "BODY"
        movement_location = max_body_location
    elif smoothed_limb_velocity > limb_threshold:
        movement_type = "LIMB"
        movement_location = max_limb_location

    return movement_type, movement_location, smoothed_body_velocity, smoothed_limb_velocity


def main():
    global prevhuman, body_threshold, limb_threshold, show_debug, center_pixels_threshold, center_time_threshold, inactivity_threshold

    # Parse command line arguments
    try:
        fn = sys.argv[1]
    except IndexError:
        fn = 0

    # Initialize video capture
    cam = cv2.VideoCapture(fn if fn != 0 else '/dev/video0')
    if not cam.isOpened():
        print("Error: Could not open camera.")
        return

    # Network settings for UDP and OSC
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    IPToSeu = "192.168.1.2"
    PORTToSeu = 7500
    client = udp_client.SimpleUDPClient(IPToSeu, PORTToSeu)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP

    last_print_time = time.time()
    curr_time = time.time()  # Initialize curr_time to avoid reference errors

    # Timers for center detection and inactivity
    center_start_time = None
    last_movement_time = time.time()
    last_pluck_time = 0
    last_reward_time = 0
    pluck_cooldown = 5.0  # Cooldown in seconds between pluck messages
    reward_cooldown = 30.0  # Cooldown in seconds between reward messages
    humans_last_seen_time = time.time()  # Track when humans were last seen
    inactivity_paused = False  # Flag to track if inactivity timer is paused
    center_timer_paused = False  # Flag to track if center timer is paused
    center_elapsed_time = 0.0  # Store elapsed center time when paused

    while True:
        ret, frame = cam.read()
        if not ret:
            break

        # Get frame dimensions and calculate center - do this early
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2

        # YOLO detection with tracking
        results = model.track(frame, save=False, persist=True, conf=0.85, classes=[0], verbose=False)

        # Extract bounding boxes for humans (class 0)
        current_boxes = []
        if results[0].boxes is not None and len(results[0].boxes) > 0:
            current_boxes = results[0].boxes.xywh.cpu().numpy().tolist()
            if not isinstance(current_boxes, list):
                # Handle case where there's only one detection
                if current_boxes.ndim == 1:
                    current_boxes = [current_boxes]
                else:
                    current_boxes = current_boxes.tolist()

        # Count humans
        num_human = len(current_boxes)
        humans_present = num_human > 0

        # Update human count if changed
        if prevhuman != num_human:
            client.send_message("/human", num_human)
            prevhuman = num_human
            print(f"Human count is now {num_human}")

        # Skip further processing if no humans detected
        if not humans_present:
            cv2.putText(frame, "No humans detected - Movement detection paused",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Draw center area for reference even when no humans present
            if show_debug:
                cv2.circle(frame, (frame_center_x, frame_center_y), center_pixels_threshold, (0, 255, 255), 2)

                # Show paused center timer value if it exists
                if center_timer_paused and center_elapsed_time > 0:
                    cv2.putText(frame, f"Center hold (paused): {center_elapsed_time:.1f}s",
                                (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Show paused inactivity timer if it exists
                if inactivity_paused:
                    elapsed_inactivity = curr_time - last_movement_time
                    cv2.putText(frame, f"Inactivity (paused): {elapsed_inactivity:.1f}s",
                                (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)

            cv2.imshow('YOLO Human Movement', frame)

            # Clear history when no humans detected
            previous_boxes = []
            detection_history.clear()
            max_velocity_history.clear()
            max_limb_velocity_history.clear()

            # Pause center timer when humans leave the frame
            if center_start_time is not None and not center_timer_paused:
                center_elapsed_time = curr_time - center_start_time
                center_timer_paused = True
                center_start_time = None  # Clear start time but save elapsed time
                print(f"Center timer paused at {center_elapsed_time:.1f} seconds")

            # Pause inactivity timer when humans leave the frame
            if not inactivity_paused:
                # Store the time when humans were last seen
                humans_last_seen_time = curr_time
                inactivity_paused = True
                print(f"Inactivity timer paused at {curr_time - last_movement_time:.1f} seconds")

            # Continue to next frame
            ch = cv2.waitKey(5)
            if ch == 27:
                break
            continue

        # Resume inactivity timer if humans have returned
        if inactivity_paused:
            # Adjust the last_movement_time to preserve elapsed time before pause
            time_diff = curr_time - humans_last_seen_time
            last_movement_time += time_diff
            inactivity_paused = False
            print("Inactivity timer resumed")

        # Resume center timer if humans have returned and it was paused
        if center_timer_paused:
            # Calculate new start time based on elapsed time
            center_start_time = curr_time - center_elapsed_time
            center_timer_paused = False
            print(f"Center timer resumed at {center_elapsed_time:.1f} seconds")

        # Calculate velocities
        body_velocities, limb_velocities = calculate_velocities(current_boxes, previous_boxes)

        # Find maximum movement and determine type
        movement_type, movement_location, smoothed_body_vel, smoothed_limb_vel = find_max_movement(
            body_velocities, limb_velocities, body_threshold, limb_threshold
        )

        # Store current boxes for next frame
        previous_boxes = current_boxes.copy()

        # Display detections
        display_frame = frame.copy()
        for box in current_boxes:
            x, y, w, h = map(int, box)
            cv2.rectangle(display_frame,
                          (int(x - w / 2), int(y - h / 2)),
                          (int(x + w / 2), int(y + h / 2)),
                          (0, 255, 0), 2)

        # Display thresholds and current values
        if show_debug:
            cv2.putText(display_frame, f"Body Threshold: {body_threshold}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(display_frame, f"Limb Threshold: {limb_threshold}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(display_frame, f"Body Velocity: {smoothed_body_vel:.2f}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(display_frame, f"Limb Velocity: {smoothed_limb_vel:.2f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            if movement_type:
                cv2.putText(display_frame, f"Movement: {movement_type}", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Draw center area for reference
            cv2.circle(display_frame, (frame_center_x, frame_center_y), center_pixels_threshold, (0, 255, 255), 2)

        # Get frame dimensions and calculate center
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2

        # Send UDP message if movement detected
        curr_time = time.time()
        if movement_type and curr_time - last_print_time >= 0.25:
            print(
                f"MAX {movement_type} VELOCITY OF {smoothed_body_vel if movement_type == 'BODY' else smoothed_limb_vel:.2f} AT {movement_location}")

            # Send UDP message
            if movement_location is not None:
                movement_int = 0 if movement_type == "BODY" else 1
                max_location_bytes = np.array(
                    [int(movement_location[0]), int(movement_location[1]), movement_int]).tobytes()
                sock.sendto(max_location_bytes, (UDP_IP, UDP_PORT))

            last_print_time = curr_time
            last_movement_time = curr_time  # Update last movement time

        # Center detection logic (for pluck message)
        if humans_present and len(current_boxes) > 0:
            # Use the first detected human for center detection
            person_x, person_y = current_boxes[0][0], current_boxes[0][1]

            # Calculate distance from center
            distance_to_center = np.sqrt((person_x - frame_center_x) ** 2 + (person_y - frame_center_y) ** 2)

            # Check if person is near center
            if distance_to_center <= center_pixels_threshold:
                # Start or continue timer
                if center_start_time is None and not center_timer_paused:
                    # Only start a fresh timer if we weren't already tracking
                    center_start_time = curr_time
                    print(f"Person centered - starting timer")

                # Calculate center duration
                center_duration = 0
                if center_start_time is not None:
                    center_duration = curr_time - center_start_time

                # Display remaining time on frame
                remaining = center_time_threshold - center_duration
                if remaining > 0:
                    cv2.putText(display_frame, f"Center hold: {remaining:.1f}s", (10, 130),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Send pluck message if timer reached and cooldown passed
                if center_duration >= center_time_threshold and curr_time - last_pluck_time >= pluck_cooldown:
                    print("PLUCK MESSAGE: User held center position for required time")
                    pluck_message = np.array([0, 0, 3]).tobytes()  # Values don't matter except the 3
                    sock.sendto(pluck_message, (UDP_IP, UDP_PORT))
                    last_pluck_time = curr_time
                    center_start_time = None  # Reset timer after sending message
                    center_elapsed_time = 0.0  # Also reset the elapsed time
            else:
                # Reset timer if person moves away from center
                if center_start_time is not None:
                    print(f"Person left center position - resetting timer")
                    center_start_time = None
                    center_elapsed_time = 0.0  # Also reset the elapsed time

        # Inactivity detection (for reward message)
        time_since_movement = curr_time - last_movement_time

        # Display inactivity time on frame if significant
        if time_since_movement > 30:  # Only show after 30 seconds of inactivity
            cv2.putText(display_frame, f"Inactivity: {time_since_movement:.1f}s", (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)

        # Send reward message if inactivity threshold reached and cooldown passed
        if time_since_movement >= inactivity_threshold and curr_time - last_reward_time >= reward_cooldown:
            print("REWARD MESSAGE: No movement detected for required time")
            reward_message = np.array([0, 0, 4]).tobytes()  # Values don't matter except the 4
            sock.sendto(reward_message, (UDP_IP, UDP_PORT))
            last_reward_time = curr_time
            last_movement_time = curr_time  # Reset inactivity timer after sending message

        # Show the frame
        cv2.imshow('YOLO Human Movement', display_frame)

        # Handle key presses
        ch = cv2.waitKey(5)
        if ch == 27:
            break
        elif ch == ord('1'):
            # Increase body threshold
            body_threshold += 5
            print(f"Body threshold: {body_threshold}")
        elif ch == ord('2'):
            # Decrease body threshold
            body_threshold = max(5, body_threshold - 5)
            print(f"Body threshold: {body_threshold}")
        elif ch == ord('3'):
            # Increase limb threshold
            limb_threshold += 5
            print(f"Limb threshold: {limb_threshold}")
        elif ch == ord('4'):
            # Decrease limb threshold
            limb_threshold = max(5, limb_threshold - 5)
            print(f"Limb threshold: {limb_threshold}")
        elif ch == ord('5'):
            # Toggle debug visualization
            show_debug = not show_debug
            print(f"Debug visualization: {'on' if show_debug else 'off'}")

    # Clean up
    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
