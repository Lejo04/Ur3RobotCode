#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 0.0
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    # Xc=row*
    # Yc=col*
    P1 = [253.8160400390625, 60.183963775634766]
    P2 = [327.3412780761719, 59.85185241699219]
    deltay = P1[1] - P2[1]
    deltax = P2[0] - P1[0]
    theta = np.arctan(deltay/deltax)
    col = col -320
    row = row -240
    col = col * np.cos(theta) - row*np.sin(theta)
    row = col * np.sin(theta) + row*np.cos(theta)
    col += -(P1[0] - 320)
    row += -(P1[1] - 240)
    factor = 0.1 / deltax
    col *= factor
    row *= factor
    col += 0.01
    row += 0.01
    return col, row
     
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
   
    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea =  True
    params.minArea = 100
    params.maxArea =1000

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    lower = ()
    upper = ()
    if color == "orange":
        lower = (0,150,180)     # orange
        upper = (13,255,255)   # orange
    else:
        lower = (20, 100, 100) # yellow
        upper = (27, 255, 255) #y ellow

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 1)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    # print(xw_yw)
    return xw_yw
