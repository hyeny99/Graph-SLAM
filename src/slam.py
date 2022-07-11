#! /usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from graph import Graph, Node, Edge
from noisy_sensor import Sensor
from copy import copy




# Is graph-based slam offline?

# 1. construct a map
# 2. scan-matching: find a loop closure
# 3. update the map by minimizing the sqr error





# Config parameters
dt = 0.01 # time between measurements

# sensor noise
toggle_noise = 1
error = 0.01 # std_dev

# map parameters
map_resolution = 0.01
map_width = 10
map_height = 10
map_width_res = int(map_width/map_resolution)
map_height_res = int(map_height/map_resolution)


# Utils 
# ----------------------------------------------------------------------------------------------------------
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    return yaw

def get_quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

def normalize_angle(angle):
    if angle < 0:
        return angle+np.ceil(-angle/(2*np.pi))*2*np.pi
    elif angle > 2*np.pi:
        return angle-(np.ceil(angle/(2*np.pi))-1)*2*np.pi

    return angle 

def define_angle(angle):
    # angle in rad
    if angle > math.radians(180):
        angle = math.radians(360) - angle
    
    return angle

# Give the previous measurements and it will count displacement
def get_noisy_odometry(x,y,yaw):
    curr_pos = rospy.wait_for_message("/odom", Odometry, timeout=None)
    curr_yaw = get_rotation(curr_pos)
    dis_x = curr_pos.pose.pose.position.x-x + random.uniform(-error, error) * toggle_noise
    dis_y = curr_pos.pose.pose.position.y-y + random.uniform(-error, error) * toggle_noise
    d_yaw = curr_yaw-yaw + random.uniform(-error, error) * toggle_noise

    return dis_x, dis_y, d_yaw

# def get_noisy_measurement(scan_msg):
#     ranges = scan_msg.ranges
#     noisy_sensor.noisy_scan = []
#     noisy_sensor.max_range = scan_msg.                                                  
#     for r in ranges:
#         noisy_r = r + random.uniform(-error, error) * toggle_noise
#         noisy_sensor.noisy_scan.append(noisy_r)
    
#     return noisy_scan


def get_index(x, y):
    return int(map_height_res * y + x)

# ----------------------------------------------------------------------------------------------------------

def initialize_map_msg():
    map = OccupancyGrid()
    map.header.frame_id = "odom"
    map.info.resolution = map_resolution
    map.info.width  = map_width_res
    map.info.height = map_height_res
    map.info.origin.position.x = -5
    map.info.origin.position.y = -5
    map.info.origin.position.z = 0
    map.info.origin.orientation.x = 0
    map.info.origin.orientation.y = 0
    map.info.origin.orientation.z = 0    
    map.info.origin.orientation.w = 1
    
    # Add initial "Unknown values" to every cell:
    for i in range(map_width_res):
        for j in range(map_height_res):
            map.data.append(50)
    
    return map

def initialize_path_msg():
    path = Path()
    path.header.frame_id = "odom"
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)
    
    return path

def build_grid_map(scan_msg, map):
    #max_angle = scan_msg.angle_max
    #min_angle = scan_msg.angle_min

    # define the angle range: -30 degrees to +30 degrees
    max_angle = 0.523599
    min_angle = -0.523599
    angle_increment = scan_msg.angle_increment

    ranges = scan_msg.ranges
    range_min = scan_msg.range_min
    range_max = scan_msg.range_max

    for i in range(len(ranges)):
        angle = define_angle(ranges[0] + angle_increment * i)
        if angle >= min_angle and angle <= max_angle:
            #print(angle)
            if ranges[i] == float('inf'):
                ranges[i] = range_max
            for beam in range(int(range_min / map_resolution), int(ranges[i] / map_resolution)):
                x = int(math.cos(angle) * beam)
                y = int(math.sin(angle) * beam)
                index = get_index(x, y)
                map.data[index] = 0
            
            if ranges[i] != range_max:
                x = math.cos(angle) * (ranges[i] / map_resolution)
                y = math.sin(angle) * (ranges[i] / map_resolution)
                index = get_index(x, y)
                map.data[index] = 100
    
    return map






# def compute_Jacobian():
#     np.matrix([[x_i, x_j, x]])


# def minimize_sqr_error():
#     while !converges:
#         num_vertices = graph.get_num_vertices()
#         b = np.zeros([num_vertices, 1])
#         H = np.zeros([3, num_vertices])
#         for c in C:
#             vertex_i = c.vertex_i
#             vertex_j = c.vertex_j
#             vertex_i_index = vertex_i.get_index()
#             vertex_j_index = vertex_j.get_index()
#             error_term = c.error
#             info_m = c.info_measurement
#             A, B = compute_Jacobian(error_term, )

#             # is the size of H matrix getting larger? I guess no
#             H[vertex_i_index, vertex_i_index] += A.transpose() * info_m * A
#             H[vertex_i_index, vertex_j_index] += A.transpose() * info_m * B
#             H[vertex_j_index, vertex_i_index] += B.transpose() * info_m * A
#             H[vertex_j_index, vertex_j_index] += B.transpose() * info_m * B

#             # compute the coefficient vector
#             b[vertex_i_index] += A.transpose() * info_m * error_term
#             b[vertex_j_index] += B.transpose() * info_m * error_term


#         # keep the first node fixed
#         H[0, 0]   


# def update_map():

#     for beam in range(int(range_min / map_resolution), (int(z / map_resolution) + 1)):
#                 beam_x = int(math.cos(angle + theta) * beam)
#                 beam_y = int(math.sin(angle + theta) * beam)
                    
#                 beam_x_t = beam_x + x_pos_t 
#                 beam_y_t = beam_y + y_pos_t
                
#                 beam_index = get_index(beam_x_t, beam_y_t)
#                 log_odds = calculate_log_odds(z_index, beam_index, copy(particle))
#                 particle.map.data[beam_index] = int(retrieve_p(log_odds) * 100)

def test_test(sensor):
    return sensor
    



if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("slam", anonymous=True)
    # Define ROS rate
    rate = rospy.Rate(1/dt)

    # Initialize a map msg
    map_msg = initialize_map_msg()

    current_position =  rospy.wait_for_message("/odom", Odometry, timeout=None)
    curr_x = current_position.pose.pose.position.x
    curr_y = current_position.pose.pose.position.y
    curr_yaw = get_rotation(current_position)

    graph = Graph()

    xPred  = np.empty((3, 1)) # state
    odom_z = np.empty((3, 1)) # odom measurement
    xCurr = np.empty((3, 1)) # curr state

    xCurr[0,0] = 0
    xCurr[1,0] = 0
    xCurr[2,0] = 0

    curr_node = Node(xCurr)
    graph.add_node(curr_node)

    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)


    while not rospy.is_shutdown():
        # Measurements (odom)
        #dx, dy, dyaw = get_odometry(curr_x, curr_y, curr_yaw)
        dx, dy, dyaw  = get_noisy_odometry(curr_x, curr_y, curr_yaw)
        
        # dist = math.sqrt(dx**2 + dy**2)
        # if dist < 0.5 and math.radians(abs(dyaw)) < 0.5:
        #     continue

        curr_x       = curr_x + dx
        curr_y       = curr_y + dy
        curr_yaw     = curr_yaw + dyaw
        
            # Measurements (laser)
        scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
        noisy_scan = Sensor(scan_msg)

        #test_test(noisy_scan)
        map = build_grid_map(noisy_scan, map_msg)


        odom_z[0,0] = dx
        odom_z[1,0] = dy
        odom_z[2,0] = dyaw

        xPred[0,0] = curr_x + dx
        xPred[1,0] = curr_y + dy
        xPred[2,0] = curr_yaw + dyaw

        # pred_node = Graph(xPred)
        # graph.add_node(pred_node)

        # edge = Edge(curr_node, pred_node, noisy_sensor)
        # graph.add_edge(edge)

        # print(graph)

        map_pub.publish(map)



        rate.sleep()


