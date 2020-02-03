#!/usr/bin/env python

"""
    HoverGames Challenge 1: Fight Fire with Flyers

    Project: Air Strategist Companion
    Author: Raul Alvarez-Torrico (raul@tecbolivia.com)
    Date: 02/02/2020

    @file http_client_node.py
    @brief Send and receive drone, fire spots and firefighter data to/from the web server
"""

import rospy
from std_msgs.msg import String, Float64, Header, Int16
from geometry_msgs.msg import Point, Quaternion, Twist
from sensor_msgs.msg import NavSatFix, Temperature

from mavros_msgs.msg import VFR_HUD, WaypointList, Waypoint

import requests
import time
from xml.etree import ElementTree

import json
from dicttoxml import dicttoxml
import rospkg

from random import random

previous_time = 0
current_time = 0
previous_time = time.time()
HTTP_REQ_INTERVAL = 3

fire_pos_x = 0
fire_pos_y = 0
fire_pos_z = 0

fire_polygons_json_string = ""
fire_polygons_area_json_string = ""
fire_polygons_img_coord_dict = {}
fire_polygons_area_dict = {}
fire_pos_cart_coord_dict = {}
drone_gps_coord_dict = {}
drone_rel_altitude_dict = {}
modified_image_size_dict = {}
telemetry_dict = {}
fire_temp_dict = {}
vfr_hud_dict = {}
xml_string = ""

rospack = None

# Receives the VFR_HUD message to obtain the drone's ground speed
def vfr_hud_callback(msg):
    global vfr_hud_dict

    # print('msg.groundspeed: ', msg.groundspeed)

    vfr_hud_dict = {'airspeed':msg.airspeed,
                    'groundspeed': msg.groundspeed,
                    'heading': msg.heading,
                    'throttle': msg.throttle,
                    'altitude': msg.altitude,
                    'climb': msg.climb}

# Receives the ground temperature read from the MLX90614 sensor
def temperature_callback(msg):
    global fire_temp_dict

    fire_temp_dict = {'fire_temp': msg.temperature}

# Receives the drone's relative altitude w.r.t. ground
def rel_alt_callback(msg):
    global drone_rel_altitude_dict

    drone_rel_altitude_dict = {'drone_rel_alt': msg.data}

# Receives the image size of the frame used in the fire detection algorithm
def image_size_callback(msg):
    global modified_image_size_dict

    im_size = msg.data.split(',')
    # print('image_width=' + str(im_size[0]) + ' | image_height=' + str(im_size[1]))

    # Convert pose Point to dictionary
    modified_image_size_dict = {'image_width': im_size[0], 'image_height': im_size[1]}
    
# JUST FOR DEBUGGING:
def output_image_callback(msg):
    print('output_image_callback...')

# This comes as a dictionary converted to JSON. No need to convert to dictionary
def fire_polygons_area_callback(msg):
    global fire_polygons_area_json_string
    global fire_polygons_area_dict

    fire_polygons_area_json_string = msg.data

    fire_polygons_area_dict = json.loads(fire_polygons_area_json_string)

# This comes as a list converted to JSON, it is converted to dictionary here:
def fire_polygons_callback(msg):
    global fire_polygons_json_string
    global fire_polygons_img_coord_dict

    # print("fire_polygons_callback: " + str(msg))
    fire_polygons_json_string = msg.data
    # Your JSON is an array with a single object inside, so when you read it in you get 
    #a list with a dictionary inside. You can access your dictionary by accessing item 0 
    #in the list, as shown below:

    fire_polygons_list = json.loads(fire_polygons_json_string)
    # print("fire_polygons_list[0] " + str(fire_polygons_list[0]))
    # print("fire_polygons_list[1] " + str(fire_polygons_list[1]))
    # print("fire_polygons_list[2] " + str(fire_polygons_list[2]))

    # Convert JSON string to dictionary
    len(fire_polygons_list)
    index = len(fire_polygons_list) 

    for poly in fire_polygons_list:
        key_name = "poly_" + str(index)
        # print(key_name)
        fire_polygons_img_coord_dict[key_name] = poly
        index = index - 1
        # print(fire_polygons_img_coord_dict[key_name])

    # print(fire_polygons_img_coord_dict)

# Receives the fire pattern centroid's coordinates
def fire_pos_callback(msg):
    global previous_time
    global current_time
    global fire_pos_cart_coord_dict

    # print(msg)

    fire_pos_x = msg.x
    fire_pos_y = msg.y
    fire_pos_z = msg.z

    # Convert pose Point to dictionary
    fire_pos_cart_coord_dict = {'fire_pos_x': msg.x, 'fire_pos_y': msg.y, 'fire_pos_z': msg.z}
    # print(fire_pos_cart_coord_dict)


# drone_lat = 0
# drone_lon = 0
def global_pos_callback(msg):
    # global drone_lat
    # global drone_lon
    global drone_gps_coord_dict
    
    # drone_lat = msg.latitude
    # drone_lon = msg.longitude

    drone_gps_coord_dict = {'drone_latitude': msg.latitude, 
                            'drone_longitude': msg.longitude, 'drone_altitude': msg.altitude}
    # print(drone_gps_coord_dict)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
    # print(rospy.get_caller_id() + ": global_pos_callback " + str(msg))

    # print(rospy.get_caller_id() + ": global_pos_callback " + str(msg.status))

    # print("drone_lat: " + str(drone_lat) + " | " + "drone_lon: " + str(drone_lon))

def make_xml_string():
    global fire_polygons_img_coord_dict
    global fire_polygons_area_dict
    global fire_pos_cart_coord_dict
    global drone_gps_coord_dict
    global drone_rel_altitude_dict
    global modified_image_size_dict
    global xml_string
    global fire_temp_dict
    global vfr_hud_dict

    # # Empty dictionaries evaluate to False in Python: not dct is True
    # if not drone_gps_coord_dict:
    #     telemetry_dict['drone_gps_coord'] = drone_gps_coord_dict

    # if not fire_pos_cart_coord_dict:
    #     telemetry_dict['fire_pos_cart_coord'] = fire_pos_cart_coord_dict

    # if not fire_polygons_img_coord_dict:
    #     telemetry_dict['fire_polygons_img_coord'] = fire_polygons_img_coord_dict
    
    telemetry_dict['drone_gps_coord'] = drone_gps_coord_dict
    telemetry_dict['drone_rel_altitude'] = drone_rel_altitude_dict
    telemetry_dict['fire_pos_cart_coord'] = fire_pos_cart_coord_dict
    telemetry_dict['fire_polygons_img_coord'] = fire_polygons_img_coord_dict
    telemetry_dict['modified_image_size'] = modified_image_size_dict
    telemetry_dict['fire_temp'] = fire_temp_dict
    telemetry_dict['fire_polygons_area'] = fire_polygons_area_dict
    telemetry_dict['vfr_hud'] = vfr_hud_dict

    # print('----------------------------')
    # print(telemetry_dict)
    # print('----------------------------')
    
    xml_string = dicttoxml(telemetry_dict)

    # print('----------------------------')
    # print(xml_string)
    # print('----------------------------')

def make_http_request():
    global previous_time
    global current_time
    # global drone_lat
    # global drone_lon
    global fire_polygons_json_string
    global rospack

    # Upload data to server every HTTP_REQ_INTERVAL
    # current_time = time.time()
    # if (current_time - previous_time) > HTTP_REQ_INTERVAL:
    #     previous_time = time.time()

    # 'if True' because still checking ig the rospy.Timer works well. Check this line:
    # rospy.Timer(rospy.Duration(HTTP_REQ_INTERVAL), http_post_callback)
    # TODO: in the future remove 'if True' if the timer works as expected.
    if True:
        # Make HTTP POST requests to upload raw XML telemetry data to server 
        print('[HTTP CLIENT] Sending POST telemetry XML request...')
        try:
            global xml_string
            global fire_pos_cart_coord_dict
            # print("Sending drone telemetry XML file...")
            # print("xml_string: " + xml_string)
            print("fire_pos_cart_coord_dict: " + str(fire_pos_cart_coord_dict))

            headers = {'User-Agent': "Mozilla Firefox", 'Content-type': 'application/xml', 
                        'Accept': 'text/plain'}
            r = requests.post('http://tec.bo/airstrategist/receive_telemetry_xml.php', 
                                data=xml_string, headers=headers)

            # print(r.request.body)
            # Solo para depuracion: imprimir el codigo de estado enviado por el servidor web
            print(r.status_code, r.reason)
            # Solo para depuracion: imprimir los primeros 800 caracteres del cuerpo de la respuesta HTTP
            # print(r.text[:1500] + '...')
            # print(r.text)

        except Exception:
            print('Exception: XML HTTP POST error!')

        # Make HTTP POST requests to upload images to server
        # opencv_tracking_dir = rospack.get_path('opencv_tracking')
        # print('opencv_tracking_dir:' + str(opencv_tracking_dir))
        print('[HTTP CLIENT] Sending POST fire_tracking_frame.jpg request...')
        try:
            opencv_tracking_dir = rospack.get_path('opencv_tracking')
            print('opencv_tracking_dir:' + str(opencv_tracking_dir))
            # files = {'upload_file': open(opencv_tracking_dir + '/scripts/fire_tracking_frame.jpg', 'rb')}

            files = {'media': open(opencv_tracking_dir + '/scripts/fire_tracking_frame.jpg', 'rb')}
            headers = {'User-Agent': "Mozilla Firefox"}
            r = requests.post('http://tec.bo/airstrategist/receive_image.php', 
                                files=files, headers=headers)

            # Solo para depuracion: imprimir el codigo de estado enviado por el servidor web
            print(r.status_code, r.reason)

            # print(r.text[:1500] + '...')
            # print(r.text)

        except Exception:
            print('Exception: image HTTP POST error!')

        print('[HTTP CLIENT] Sending GET commands.xml request...')
        try:
            global task_mode_cmd_pub
            global firefighter_current_seq
            global sel_crew_member_idx_pub

            headers = {'User-Agent': "Mozilla Firefox"}
            r = requests.get('http://tec.bo/airstrategist/commands.xml', headers=headers)

            # Solo para depuracion: imprimir el codigo de estado enviado por el servidor web
            print(r.status_code, r.reason)

            # print(r.text[:1500] + '...')
            # print(r.text)

            # tree = ElementTree.fromstring(response.content)
            root = ElementTree.fromstring(r.content)
            # # JUST FOR DEBUGGING:
            # for child in root.iter('*'):
            #     print(child.tag, child.attrib)

            cmd_value = ""
            sel_crew_mem = ""
            firefighter_current_seq = ""
            for task_mode_cmd in root.findall('task_mode_cmd'):
                # unix_time = task_mode_cmd.find('unix_time').text
                unix_time = task_mode_cmd.get('unix_time')
                cmd_value = task_mode_cmd.get('cmd_value')
                # 'sel_crew_member_idx' can be absent if there's no selected crew member from the web page
                # it affects to the following two variables:
                sel_crew_mem = task_mode_cmd.get('sel_crew_member_idx')
                firefighter_current_seq = task_mode_cmd.get('sel_crew_member_idx')
                # print('unix_time, cmd_value: ' + unix_time + ', ' + cmd_value)

            try:
                task_mode_cmd_pub.publish(int(cmd_value))
            except Exception as e:
                print('Exception task_mode_cmd_pub.publish: ', e)

            try:
                if sel_crew_mem: # If the string is not ""
                    sel_crew_member_idx_pub.publish(int(sel_crew_mem))
            except Exception as e:
                print('Exception sel_crew_member_idx_pub.publish: ', e)

        except Exception:
            print('Exception: GET commands.xml error!')

        print('[HTTP CLIENT] Sending GET dump_fighter_data_xml.php request...')
        try:
            global fighter_crew_list_pub
            global firefighter_current_seq
            headers = {'User-Agent': "Mozilla Firefox"}
            r = requests.get('http://tec.bo/airstrategist/fighter_data/dump_fighter_data_xml.php', headers=headers)

            # Solo para depuracion: imprimir el codigo de estado enviado por el servidor web
            print(r.status_code, r.reason)

            # print(r.text[:1500] + '...')
            # print(r.text) # Print the same as print(r.content)
            # print(r.content)

            root = ElementTree.fromstring(r.content)
            # # JUST FOR DEBUGGING:
            # for child in root.iter('*'):
            #     print(child.tag, child.attrib)

            fighter_crew_list = WaypointList()
            fighter_crew_list2 = []
            
            idx = 1
            # firefighter_current_seq = 2 # For debug
            for fighter in root.findall('fighter'):
                # unix_time = task_mode_cmd.find('unix_time').text
                fighter_name = fighter.get('name')
                fighter_latitude = fighter.get('latitude')
                fighter_longitude = fighter.get('longitude')

                # print('fighter_name, fighter_latitude, fighter_longitude: ' + fighter_name + ', ' + fighter_latitude + ', ' + fighter_longitude)

                # fighter_crew_list[idx].latitude = fighter_latitude
                # fighter_crew_list[idx].longitude = fighter_longitude

                wp_msg = Waypoint()
                wp_msg.frame = 0; # mavros_msgs::Waypoint::FRAME_GLOBAL;
                wp_msg.command = 16;
                # print('idx, firefighter_current_seq: ' + str(idx) + firefighter_current_seq)
                
                wp_msg.is_current = False;

                # 'sel_crew_member_idx' can be absent if there's no selected crew member from the web page
                # it affects the value of firefighter_current_seq
                # Check if firefighter_current_seq is not ""
                if firefighter_current_seq: 
                    if int(firefighter_current_seq) == idx:
                        wp_msg.is_current = True
                # If "", give it a temporary value of "0" until a crew member is selected from the web page
                else:
                    firefighter_current_seq = "0"
                # if int(firefighter_current_seq) == idx:
                #     wp_msg.is_current = True
                # else:
                #     wp_msg.is_current = False;

                wp_msg.autocontinue = False;
                wp_msg.param1 = 0;
                wp_msg.param2 = 0;
                wp_msg.param3 = 0;
                wp_msg.param4 = 0;
                wp_msg.x_lat = float(fighter_latitude)
                wp_msg.y_long = float(fighter_longitude)
                # p_msg.x_lat = 40.0 + idx + random();
                # wp_msg.y_long = 30.0  + idx + random();
                wp_msg.z_alt = 5.0;

                idx = idx + 1

                # fighter_crew_list.append(wp_msg)
                # fighter_crew_list2[idx] = wp_msg
                fighter_crew_list2.append(wp_msg)
            # print('fighter_crew_list2: ', fighter_crew_list2)
            try:
                # fighter_crew_list_pub.publish(fighter_crew_list)
                fighter_crew_list_pub.publish(int(firefighter_current_seq), fighter_crew_list2)
            except Exception as e:
                # print(e)
                print('Exception fighter_crew_list_pub.publish: ', e)

        # except Exception:
        #     print('Exception: GET dump_fighter_data_xml.php error!')

        except Exception as e:
            # print(e)
            print('Exception: GET dump_fighter_data_xml.php error!: ', e)

def http_post_callback(event):
    print('\r\n')
    # print 'Timer called at ' + str(event.current_real)

    make_xml_string()
    make_http_request()


def listener():
    global rospack
    global task_mode_cmd_pub
    global fighter_crew_list_pub
    global sel_crew_member_idx_pub

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # -------------------- Create ROS subscribers --------------------
    # Subscribe to the fire pattern centroid's local Cartesian coordinates
    rospy.Subscriber("/fire_pattern/pose_point", Point, fire_pos_callback)

    # Subscribe to obtain global latitude, longitude and altitude
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_pos_callback)

    # Subscribe to obtain relative altitude (w.r.t. ground)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, rel_alt_callback)

    # Subscribe to obtain the drone's ground speed, along with atoher data that could be
    # useful in the future
    rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, vfr_hud_callback)

    # Subscribe to obtain the vertices of all fire polygons in local Cartesian coordinates
    rospy.Subscriber("/fire_polygons_json", String, fire_polygons_callback)

    # Subscribe to obtain the areas of all fire polygons in squared pixel units
    rospy.Subscriber("/fire_polygons_area_json", String, fire_polygons_area_callback)

    # Subscribe to obtain the image size in pixels used in the detection process. Useful for
    # Calculatig the conversion factor from pixel units to meters
    rospy.Subscriber("/modified_image/size", String, image_size_callback)


    # Subscribe to obtain the ground temperature taken from the air
    rospy.Subscriber("/mlx90614/temp", Temperature, temperature_callback)

    # -------------------- Create ROS publishers --------------------
    # Publishes the task mode commands received from the web page's control pane
    task_mode_cmd_pub = rospy.Publisher("/drone_commands/task_mode_cmd", Int16, queue_size=10)

    # Publishes the index of the current selected firefighter to be tracked
    # received from the web page's control pane
    sel_crew_member_idx_pub = rospy.Publisher("/drone_commands/sel_crew_member_idx", Int16, queue_size=10)

    # Publishes the coordinates of all firefighters currently detected by the sistem
    # received from the web page's control pane
    fighter_crew_list_pub = rospy.Publisher("/drone_commands/fighter_crew_list", WaypointList, queue_size=10)
    
    # Timer to send the HTTP requests every few seconds
    rospy.Timer(rospy.Duration(HTTP_REQ_INTERVAL), http_post_callback)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    listener()