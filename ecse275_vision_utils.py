#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 22 17:46:49 2023

@author: zxc703
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import copy
import cv2

def hand_eye_transform(pos_cam, T_cam_world):
    
    '''
    Transform a position from a camera coordinate system to a world coordinate system using a given transformation matrix.
    
    Args:
        pos_cam (numpy.ndarray): A 1D NumPy array representing a position in the camera coordinate system (e.g., [x, y, z]).
        T_cam_world (numpy.ndarray): A 3x4 transformation matrix representing the transformation from camera to world coordinates.
    
    Returns:
        numpy.ndarray: A 1D NumPy array representing the transformed position in the world coordinate system.
    '''
    
    
    pos_world = np.dot(T_cam_world[:3,:3],pos_cam) + T_cam_world[:,3]
    
    return pos_world
    
def move_to(sim,desired_pos,offset=0.01,approach_height=0.1,wait_time=3):
    '''
    Move a robot to a desired position while implementing an approach and optional offset.

    Args:
        desired_pos (list): A list representing the desired position in 3D space (x, y, z) to which the robot should move.
        offset (float, optional): An offset value added to the desired z-coordinate to create a relative position for the robot.
        approach_height (float, optional): The height above the desired position to approach before reaching it.
        wait_time (float, optional): The maximum simulation time (in seconds) to wait for the robot to reach the desired position.
    
    Returns:
        None
    '''
    start_time = sim.getSimulationTime()
    approach_pos = copy.copy(desired_pos)
    approach_pos[2] = approach_pos[2]+approach_height
    desired_pos[2] = desired_pos[2]+offset
    sim.callScriptFunction('set_desired_pose',sim.getScript(1,sim.getObject('/Franka')),approach_pos)
    print("position for approach")
    while sim.getSimulationTime()-start_time < wait_time:
        pass
    start_time = sim.getSimulationTime()
    print("moving to target")
    sim.callScriptFunction('set_desired_pose',sim.getScript(1,sim.getObject('/Franka')),desired_pos)
    while sim.getSimulationTime()-start_time < wait_time:
        pass
    print("movement_completed")
    
   
def toggle_gripper(sim,wait_time=2):
    '''
    Toggle the gripper of a robot and wait for a specified duration.

    Args:
        wait_time (float, optional): The time (in seconds) to wait after toggling the gripper.
    
    Returns:
        None
    
    '''
    start_time = sim.getSimulationTime()
    print('toggling gripper')
    sim.callScriptFunction('toggle_gripper',sim.getScript(1,sim.getObject('/Franka/FrankaGripper')))
    while sim.getSimulationTime()-start_time < wait_time:
        pass

def detect_blobs(image,visualize=False):
    '''Uses open cv to detect blobs in an image and return a special keypoints iterable object'''
    params = cv2.SimpleBlobDetector_Params()

    # Filter by color (binary image)
    params.filterByColor = False
    params.blobColor = 0  # 0 means black blobs

    # Filter by area (area in pixels)
    params.filterByArea = True
    params.minArea = 5# Minimum blob area

    # Filter by circularity (0 to 1)
    params.filterByCircularity = False
    params.minCircularity = 0.8  # Minimum circularity

    params.minThreshold =  0
    params.maxThreshold =  80

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(image)
    result_image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if visualize:
        plt.figure()
        plt.imshow(result_image,cmap="binary")

    return keypoints

def blob_images(image,keypoints,roi_size=15):
    '''Returns a list of 2D pixel arrays of the regions of interest in a given image as specified by the keypoints, the regions are squares whose size is determined via roi size'''
    roi = []
    centroids = []
    masked_image = mask_image(image)
    for kp in keypoints:
        x, y = map(int, kp.pt)
        half_size = roi_size // 2
        cropped_img = masked_image[y - half_size:y + half_size, x - half_size:x + half_size]
        cropped_img[np.where(cropped_img<55)] = 0
        roi.append(cropped_img)
        centroids.append((x,y))

    return centroids,roi

def mask_image(image):
    '''Function to mask green background into black'''
    mask = np.where(image==100)
    masked_image = copy.copy(image)
    masked_image[mask[0],mask[1]] = 0

    return masked_image # return the inversion of the image

def process_image(image):

    keypoints = detect_blobs(image,visualize=True)
    centroids,list_of_images = blob_images(image,keypoints,18)

    # resize the image to fit with MNIST
    resized_images = []
    for image in list_of_images:
        resized_images.append(cv2.resize(image, (28, 28)))

    return centroids, resized_images