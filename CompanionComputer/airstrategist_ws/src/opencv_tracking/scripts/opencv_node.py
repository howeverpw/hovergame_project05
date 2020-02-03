#!/usr/bin/env python

"""
    HoverGames Challenge 1: Fight Fire with Flyers

    Project: Air Strategist Companion
    Author: Raul Alvarez-Torrico (raul@tecbolivia.com)
    Date: 02/02/2020

    @file opencv_node.py
    @brief Detects fire spots with an infrared camera
"""

from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String, MultiArrayLayout, UInt16MultiArray
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from time import sleep

import codecs, json
import os
import rospkg

class FireDetector:

    def __init__(self):

        # Constants and variables
        # Use dividers of two to get a more convenient image height and width, such as: 240x320
        self.IMAGE_RESIZE_PERCENT = 0.25 #0.5 

        self.point_msg = Point(0, 0, 0)
        self.quaternion_msg = Quaternion(0, 0, 0, 0)

        self.cx_blob_cam_coord_array = []
        self.cy_blob_cam_coord_array = []

        self.modified_image_width = 0
        self.modified_image_height = 0

        self.approx_array_json_string = ""
        self.fire_area_json_string = ""
        self.output_frame_to_file = None

        self.bridge = CvBridge()

        # -------------------- Create ROS publishers --------------------
        # Publishes the concatenation of the detection video frame and the masked video frame
        self.fire_detect_concat_image_pub = rospy.Publisher("/fire_detect_concat_image/compressed", 
                         CompressedImage, queue_size=10)

        # Publishes the detection video frame
        self.fire_tracking_image_pub = rospy.Publisher("/fire_tracking_image/compressed", 
                         CompressedImage, queue_size=10)

        # Publishes the fire pattern centroid's local Cartesian coordinates
        self.pose_point_pub = rospy.Publisher("/fire_pattern/pose_point", Point, queue_size=10)

        # Publishes the fire pattern centroid's pose quaternion (orientation)
        # NOTE: Not implemented yet.
        self.pose_quaternion_pub = rospy.Publisher("/fire_pattern/pose_quaternion", Quaternion, queue_size=10)

        # Publishes the local Cartesian coordinates of Fire Spots 1, 2, 3
        self.fire_spot1_point_pub = rospy.Publisher("/fire_pattern/fire_spot1_pose_point", Point, queue_size=10)
        self.fire_spot2_point_pub = rospy.Publisher("/fire_pattern/fire_spot2_pose_point", Point, queue_size=10)
        self.fire_spot3_point_pub = rospy.Publisher("/fire_pattern/fire_spot3_pose_point", Point, queue_size=10)

        # Publishes the image size in pixels used in the detection process. Useful for
        # Calculatig the conversion factor from pixel units to meters
        self.image_size_pub = rospy.Publisher("/modified_image/size", String, queue_size=10)
        # TODO: use array of floats instead os a string
        # self.image_size_pub = rospy.Publisher("/modified_image/size", MultiArrayLayout, queue_size=10)

        # Publishes the vertices of all fire polygons in local Cartesian coordinates
        self.fire_polygons_json_pub = rospy.Publisher("/fire_polygons_json", String, queue_size=10)

        # Publishes the areas of all fire polygons in squared pixel units
        self.fire_area_json_pub = rospy.Publisher("/fire_polygons_area_json", String, queue_size=10)

        # Get an instance of RosPack with the default search paths
        self.rospack = rospkg.RosPack()

        # -------------------- Create ROS subscribers --------------------
        # Subcribes to the Raspberry Pi camera's image feed
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", 
                         CompressedImage, self.image_callback)

        # Timer to control the saving of the detection frame every few seconds
        # (this frames shows up at the web page)
        self.save_detection_frame_timer = rospy.Timer(rospy.Duration(3), self.save_detection_frame_callback)

    # ---------------- Saves the detection frame every few seconds ----------------
    def save_detection_frame_callback(self, event):
        # print('\r\n')
        # print('save_detection_frame_callback called at ' + str(event.current_real))
        
        # WARNING: file will be saved in /home/pi/.ros (hidden dir) if full path not specified
        # check it with cwd = os.getcwd()
        opencv_tracking_dir = self.rospack.get_path('opencv_tracking')
        ret_val = cv2.imwrite(opencv_tracking_dir + '/scripts/fire_tracking_frame.jpg', self.output_frame_to_file)

        if not ret_val:
            print('[OPENCV NODE] Error while saving camera shot!')
        else:
            print('[OPENCV NODE] Camera shot saved...')

    # ---------------- Processes frames from the raspicam_node image feed ----------------
    # The fire spots are detected in this function
    def image_callback(self, camera_frame):
        try:
            # Get passed image from ROS topic
            pass_image = self.bridge.compressed_imgmsg_to_cv2(camera_frame, "passthrough")

        except CvBridgeError as e:
            print(e)

        # ---- I used these with RGB cameras, could be useful with a thermal camera (no fully tested)
        # ---- RGB image, BGR color space ----
        # blurred_cam_frame = cv2.GaussianBlur(camera_frame, (21, 21), 0)
        # hsv_cam_frame = cv2.cvtColor(blurred_cam_frame, cv2.COLOR_BGR2HSV)
        # lower = [18, 50, 50]
        # upper = [35, 255, 255]

        # Con infrarrojo:
        # Con infrarrojo:
        
        # lower = [268*255/360, 12*255/100, 100*255/100]
        # upper = [360*255/360, 0*255/100, 100*255/100]
        # lower = [303*255/360, 26*255/100, 100*255/100]
        # upper = [360*255/360, 0*255/100, 100*255/100]
        # --------------------------------------------------------------------------------------------

        # ---- IR image, BGR color space ----
        # Resize the original camera frame to a smaller size (no need to overload the companion computer)
        resized_frame = cv2.resize(pass_image, (0,0), fx=self.IMAGE_RESIZE_PERCENT, fy=self.IMAGE_RESIZE_PERCENT)

        # Get new image size to publish as ROS topic
        self.modified_image_height, self.modified_image_width, channels = resized_frame.shape

        # Get new image size for local calculations
        # TODO: Change code to use just othe other set of size variables:
        # (self.modified_image_height, self.modified_image_width)
        resized_height, resized_width, resized_channels = resized_frame.shape
        blurred_cam_frame = cv2.GaussianBlur(resized_frame, (5, 5), 0)

        # ---- HSV conversion ----
        # NOTE: HSV could be useful in future versions, not used currently, not tested in detail
        hsv_cam_frame = blurred_cam_frame # Bypass HSV conversion
        
        # hsv_cam_frame = resized_frame

        # Mask for IR image
        lower = [255, 120, 255] # "Light Pink" for IR images 221, 97, 0
        upper = [255, 255, 255]

        # Mask for BGR image
        # lower = [221, 97, 0]
        # lower = [0, 20, 211] # Orange 
        # upper = [255, 255, 255]

        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(hsv_cam_frame, lower, upper)
        greenLower = (255, 120, 255)
        greenUpper = (255, 120, 255)
        # mask = cv2.inRange(hsv_cam_frame, greenLower, greenUpper)
        # --------------------------------------------------------------------------

        # Mask image with detected fire pixels
        # masked_frame = cv2.bitwise_and(resized_frame, hsv_cam_frame, mask=mask) # Use HSV conversion
        masked_frame = cv2.bitwise_and(resized_frame, blurred_cam_frame, mask=mask) # Bypass HSV conversion

        ### ----- Print x, y coordinates in the image -----
        fire_tracking_frame = resized_frame.copy()

        x_axe_pos = int(resized_height/2 - 1)
        y_axe_pos = int(resized_width/2 - 1)

        cv2.line(fire_tracking_frame, (0, x_axe_pos), (resized_width, x_axe_pos), (0, 128, 255), 1)
        cv2.line(fire_tracking_frame, (y_axe_pos, 0), (y_axe_pos, resized_height), (0, 128, 255), 1)
        cv2.circle(fire_tracking_frame, (y_axe_pos, x_axe_pos), 1, (255, 255, 255), -1)

        ### -----  Draw contours -----
        gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        #set a thresh
        thresh = 200
        #get threshold image
        ret, thresh_frame = cv2.threshold(gray_frame, thresh, 255, cv2.THRESH_BINARY)
        #find contours
        contours_frame, contours, hierarchy = cv2.findContours(thresh_frame, 
            cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # cv2.RETR_LIST ?

        # Find the total contour number and the biggest contour area
        # The max contour area is just for debugging
        max_contour_area = 0
        num_contours = 0
        for contour in contours:
            num_contours = num_contours + 1
            # area = cv2.contourArea(contour)
            # if area > max_contour_area:
            #     max_contour_area = area

        # print("max_contour_area:", max_contour_area)
        #create an empty image for contours
        img_contours = np.zeros(resized_frame.shape)
        
        # draw the contours on the empty image
        cv2.drawContours(fire_tracking_frame, contours, -1, (0, 255, 0), 2)

        # ordered list of contours in increasing order w.r.t area
        contours_sorted = sorted(contours, key=cv2.contourArea)
        # print('num_contours=' + str(num_contours))

        # Draw polygon aproximation of the MAX_NUM_BIGGEST_CONTOURS
        MAX_NUM_BIGGEST_CONTOURS = 3
        MAX_POLY_POINTS = 10
        
        approx_poly_array = [] # To store the MAX_NUM_BIGGEST_CONTOURS approx polynomials
        approx_poly_cartesian_list_array = [] # To store approx polynomials converted to lists for JSON serialization
        fire_spot_area_dict = {}

        num_biggest_fire_spots = 0
        if num_contours > MAX_NUM_BIGGEST_CONTOURS:
            num_biggest_fire_spots = MAX_NUM_BIGGEST_CONTOURS
        else:
            num_biggest_fire_spots = num_contours

        counter = num_biggest_fire_spots

        x_blobs_cart_coord_array = []
        y_blobs_cart_coord_array = []

        # for contour in contours_sorted[-MAX_NUM_BIGGEST_CONTOURS:]:
        for contour in contours_sorted[-num_biggest_fire_spots:]:
            # Calculate contour area
            contour_area = cv2.contourArea(contour)

            #### ------- Calculate cartesian coordinates for each fire spot --------

            ## -------- Cartesina coordinates for Fire Spot 1 --------
            # calculate moments of binary image
            M_1 = cv2.moments(contour)
            # calculate x,y coordinate of center
            cx_blob1_cam_coord = 0
            cy_blob1_cam_coord = 0
            
            ## -------- Check for fire blobs --------
            # Fire blobs have been found:
            if M_1["m00"] > 0:
                # print("All contours area:", M_1["m00"])

                cx_blob1_cam_coord = int(M_1["m10"] / M_1["m00"])
                cy_blob1_cam_coord = int(M_1["m01"] / M_1["m00"])

                # Cartesian coordinates of unfiltered blob center (just for visualization)
                x_blobs_cart_coord_array.append(cx_blob1_cam_coord - y_axe_pos)
                y_blobs_cart_coord_array.append(x_axe_pos - cy_blob1_cam_coord)

            # No fire blobs found:
            else:
                # All cartesian coordinates to zero
                x_blobs_cart_coord_array.append(0)
                y_blobs_cart_coord_array.append(0)
                # Camera coordinates to cartesian coordinate origin
                cx_blob1_cam_coord = y_axe_pos
                cy_blob1_cam_coord = x_axe_pos
            ## END: -------- Cartesina coordinates for Fire Spot 1 --------

            ### -- Compute the approximation polynomial for the current contour
            coef = 0.02 # 0.03->7 ppoints, 0.02->9 points, 0.04-> 5 points
            epsilon = coef * cv2.arcLength(contour, True)
            approx_poly = cv2.approxPolyDP(contour, epsilon, closed=True)
            # print("shape size [1 is largest]: " + str(counter) + " : " + str(approx_poly.shape) 
            #               + " vertices, coef=" + str(coef))

            # # ---- Find a poly approximation with num. of points <= points_wanted ----------------
            # # Works in the PC, not in the RPi! (don't know why)
            # #cnt = Your contour to be approximated
            # cnt = contour
            # points_wanted = 10

            # # Make precision bigger to make approx_poly.shape[0] tend to points_wanted
            # # but it'll require more iterations on average
            # precision = 100 #10000 
            # # print("contour: ", contour)

            # approx_poly_found = False
            # for x in range(precision):
            # # for x in reversed(range(1, precision)):
            #     coef = x/precision
            #     epsilon = coef*cv2.arcLength(cnt, True)
            #     approx_poly = cv2.approxPolyDP(cnt, epsilon, True)
            #     if approx_poly.shape[0] <= points_wanted:
            #         # Print 
            #         print("shape size [1 is largest]: " + str(counter) + " : " + str(approx_poly.shape) 
            #              + " vertices | x=" + str(x)+ ", coef=" + str(coef))
            #         # print("approx_poly: ", approx_poly)
            #         # print("approx_poly.tolist(): ", approx_poly.tolist())
            #         fire_polygons_json_string = json.dumps(approx_poly.tolist())
            #         # print("fire_polygons_json_string: ", fire_polygons_json_string)
            #         approx_poly_found = True
            #         break
            
            # if approx_poly_found == False:
            #     print("No poly approximation found!")
            
            ## At this point, approx_poly is the np.array containing the approxPolyDP 
            ## corresponding to current contour
            # Append approx polynomials
            # print("approx_poly: " + str(approx_poly))
            # print("approx_poly.shape[0]: " + str(approx_poly.shape[0]))
            ## -----------------------------------------------------------------------------------------

            approx_poly_num_points = approx_poly.shape[0]; # Num. point of the polynomial
            # Create another array for the result of converting from 'pixel' cordinates
            # to local cartesian coordinates
            approx_poly_cartesian = np.copy(approx_poly) 

            # Convert from pixel to cartesian coordinates
            for index in range(approx_poly_num_points):
                approx_poly_cartesian[index][0][0] = approx_poly[index][0][0]  - y_axe_pos
                approx_poly_cartesian[index][0][1] = x_axe_pos - approx_poly[index][0][1]

            # print("resized_height=" + str(resized_height) + " | resized_width=" + str(resized_width))
            # print("y_axe_pos=" + str(y_axe_pos) + " | x_axe_pos=" + str(x_axe_pos))
            # print("approx_poly_cartesian: " + str(approx_poly_cartesian))

            # Append approximation polynomial (in pixels) to array for drawing it in the frame
            approx_poly_array.append(approx_poly) 
            

            # Append approx polynomials CONVERTED TO CARTESIAN, to lists for JSON serialization
            # Reshape to store coordinates (x, y) consecutively in the order: x1, y2, x2, y2, x3, y3, ...
            approx_poly_cartesian_reshaped = np.reshape(approx_poly_cartesian, 2*approx_poly_cartesian.shape[0])
            approx_poly_cartesian_list = approx_poly_cartesian_reshaped.tolist()
            approx_poly_cartesian_list_array.append(approx_poly_cartesian_list)

            fire_spot_area_dict[counter] = contour_area


            counter = counter -1

        # Draw approx polynomials
        cv2.drawContours(fire_tracking_frame, approx_poly_array, -1, (0, 0, 255), 2)

        # Serialize approx polynomials and areas to JSON string. This are published as a ROS topic
        # This is serialized as a list (will be converted to dict in http_client_node.py):
        self.approx_array_json_string = json.dumps(approx_poly_cartesian_list_array)
        # This is serialized as a dict (no conversion needed in http_client_node.py):
        self.fire_area_json_string = json.dumps(fire_spot_area_dict)

        # JUST FOR DEBUGGING: Serialize approx polynomials to JSON file
        file_path = "fire_polygons.json" ## your path variable
        json.dump(approx_poly_cartesian_list_array, codecs.open(file_path, 'w', encoding='utf-8'), \
            separators=(',', ':'), sort_keys=True, indent=4) ### this saves the array in .json format

        ### -----  Find the contours centroid    
        # convert the grayscale image to binary image
        # ret, thresh_frame = cv2.threshold(gray_frame, 127, 255, 0)

        # calculate moments of binary image
        # M = cv2.moments(thresh_frame)
        M = cv2.moments(contours_frame)

        # calculate x,y coordinate of center
        cx_blob_cam_coord = 0
        cy_blob_cam_coord = 0
        # print("All contours area:", M["m00"])
        cv2.putText(fire_tracking_frame, "All contours area: " + str(M["m00"]), (25, resized_height - 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        ## -------- Check for fire blobs --------
        # Fire blobs have been found:
        if M["m00"] > 0:
            # print("All contours area:", M["m00"])

            cx_blob_cam_coord = int(M["m10"] / M["m00"])
            cy_blob_cam_coord = int(M["m01"] / M["m00"])

            # Filter coordinates
            num_filt_points = 10
            # Append new values to filter
            self.cx_blob_cam_coord_array.append(cx_blob_cam_coord)
            self.cy_blob_cam_coord_array.append(cy_blob_cam_coord)
            # If filter was full already (num_filt_points), discard the oldest value
            if len(self.cx_blob_cam_coord_array) > num_filt_points:
                self.cx_blob_cam_coord_array = self.cx_blob_cam_coord_array[1:]
                self.cy_blob_cam_coord_array = self.cy_blob_cam_coord_array[1:]
            
            # Compute filtered camera coordinates
            N = len(self.cx_blob_cam_coord_array)
            cX_filt_array = np.convolve(self.cx_blob_cam_coord_array, np.ones((N,))/N, mode='valid')
            cY_filt_array = np.convolve(self.cy_blob_cam_coord_array, np.ones((N,))/N, mode='valid')
            cx_filt_blob_cam_coord = int(round(sum( cX_filt_array )))
            cy_filt_blob_cam_coord = int(round(sum( cY_filt_array )))
            # print("Filt. coord: " + str(cx_filt_blob_cam_coord) + " " + str(cy_filt_blob_cam_coord))
            
            # Convert from camera coordinates to cartesian in pixel units
            x_filt_blob_cart_coord = cx_filt_blob_cam_coord - y_axe_pos
            y_filt_blob_cart_coord = x_axe_pos - cy_filt_blob_cam_coord

            # Cartesian coordinates of unfiltered blob center (just for visualization)
            x_blob_cart_coord = cx_blob_cam_coord - y_axe_pos
            y_blob_cart_coord = x_axe_pos - cy_blob_cam_coord

        # No fire blobs found:
        else:
            # All cartesian coordinates to zero
            x_blob_cart_coord = 0
            y_blob_cart_coord = 0
            x_filt_blob_cart_coord = 0
            y_filt_blob_cart_coord = 0
            # Camera coordinates to cartesian coordinate origin
            cx_blob_cam_coord = y_axe_pos
            cy_blob_cam_coord = x_axe_pos
            cx_filt_blob_cam_coord = y_axe_pos
            cy_filt_blob_cam_coord = x_axe_pos
            
        # Draw red dot and coordinates of unfiltered point
        cv2.circle(fire_tracking_frame, (cx_blob_cam_coord, cy_blob_cam_coord), 5, (0, 0, 255), -1) 
        cv2.putText(fire_tracking_frame, str(x_blob_cart_coord) + ", " + str(y_blob_cart_coord), 
            (cx_blob_cam_coord + 25, cy_blob_cam_coord - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw blut dot and coordinates of filtered point
        cv2.circle(fire_tracking_frame, (cx_filt_blob_cam_coord, cy_filt_blob_cam_coord), 5, (255, 30, 30), -1)
        cv2.putText(fire_tracking_frame, str(x_filt_blob_cart_coord) + ", " + str(y_filt_blob_cart_coord), 
            (cx_filt_blob_cam_coord + 25, cy_filt_blob_cam_coord - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 30, 30), 1)

        # Save reference for writing this frame to file
        self.output_frame_to_file = fire_tracking_frame

        fire_detect_concat_image = np.concatenate((fire_tracking_frame, masked_frame), axis=0)

        # JUST FOR DEBUGGING: Show image in window
        # cv2.imshow('fire_detect_concat_image', fire_detect_concat_image)
        cv2.waitKey(1)

        x_blobs_cart_coord_array.reverse()
        y_blobs_cart_coord_array.reverse()

        # Publish Fire Spot 1 centroid coordinates
        try:
            if len(x_blobs_cart_coord_array) >= 1:
                fire_spot1_point_msg = Point(x_blobs_cart_coord_array[0], y_blobs_cart_coord_array[0], 0)
                fire_spot1_quaternion_msg = Quaternion(x_blobs_cart_coord_array[0], y_blobs_cart_coord_array[0], 0, 0)
                self.fire_spot1_point_pub.publish(fire_spot1_point_msg)
        except Exception as e: # Fire Spot 1 wasn't detected (not found in array)
            print('Exception fire_spot1_point_pub: ', e)

        # Publish Fire Spot 2 centroid coordinates
        try:
            if len(x_blobs_cart_coord_array) >= 2:
                fire_spot2_point_msg = Point(x_blobs_cart_coord_array[1], y_blobs_cart_coord_array[1], 0)
                fire_spot2_quaternion_msg = Quaternion(x_blobs_cart_coord_array[1], y_blobs_cart_coord_array[1], 0, 0)
                self.fire_spot2_point_pub.publish(fire_spot2_point_msg)
        except Exception as e: # Fire Spot 2 wasn't detected (not found in array)
            print('Exception fire_spot2_point_pub: ', e)

        # Publish Fire Spot 3 centroid coordinates
        try:
            if len(x_blobs_cart_coord_array) >= 3:
                fire_spot3_point_msg = Point(x_blobs_cart_coord_array[2], y_blobs_cart_coord_array[2], 0)
                fire_spot3_quaternion_msg = Quaternion(x_blobs_cart_coord_array[2], y_blobs_cart_coord_array[2], 0, 0)
                self.fire_spot3_point_pub.publish(fire_spot3_point_msg)
        except Exception as e: # Fire Spot 3 wasn't detected (not found in array)
            print('Exception fire_spot3_point_pub: ', e)

        # Publish fire blobs & contour concatenated images
        try:
            self.fire_detect_concat_image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(fire_detect_concat_image))
        except CvBridgeError as e:
            print('Exception fire_detect_concat_image_pub: ', e)

        # Publish fire blobs image
        try:
            self.fire_tracking_image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(fire_tracking_frame))
        except CvBridgeError as e:
            print('Exception fire_tracking_image_pub: ', e)

        # Publish fire blobs centroid coordinates
        self.point_msg = Point(x_blob_cart_coord, y_blob_cart_coord, 0)
        try:
            self.pose_point_pub.publish(self.point_msg)
        except Exception as e:
            print('Exception pose_point_pub: ', e)

        # Publish fire blobs centroid orientation quaternion
        # NOTE: Not quite implemented yet, work in progress...
        self.quaternion_msg = Quaternion(x_blob_cart_coord, y_blob_cart_coord, 0, 0)
        try:
            self.pose_quaternion_pub.publish(self.quaternion_msg)
        except Exception as e:
            print('Exception pose_quaternion_pub: ', e)

        # Publish image height and width
        try:
            # self.modified_image_height, self.modified_image_width
            self.image_size_pub.publish(str(self.modified_image_width) + "," +  str(self.modified_image_height))
        except Exception as e:
            print('Exception image_size_pub: ', e)

        # Publish fire polygons JSON file
        try:
            self.fire_polygons_json_pub.publish(self.approx_array_json_string)
        except Exception as e:
            print('Exception fire_polygons_json_pub: ', e)

        
        # Publish fire polygons area JSON file
        try:
            self.fire_area_json_pub.publish(self.fire_area_json_string)
        except Exception as e:
            print('Exception fire_area_json_pub: ', e)
            
            # sleep(0.100)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


def main(args):
    # WARNING: Put pospy.init before the object instantiation, otherwise rospy.Timer doesn't work
    # Not sure why!
    rospy.init_node("image_converter", anonymous=True)
    ic = FireDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
