#! /usr/bin/env python3
import os
import sys
curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, curr_dir) 

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf.transformations import euler_from_quaternion
from copy import copy, deepcopy
from front_end.graph import Vertex, Edge

from sensor.noisy_sensor import Noisy_sensor
import sensor.noisy_odom as noisy_odom
import scan_matching.icp as icp 


#from test.icp_test import plot, plot_poses
from scan_matching.loop_detection import Loop_closure
from back_end.graph_optimization import is_converged, optimize_graph, plot_path
from front_end import graph_build





# Config parameters
dt = 0.01 # time between measurements

map_resolution = 0.01
map_x = 30
map_y = 30

grid_dim_x = int(map_x / map_resolution)
grid_dim_y = int(map_y / map_resolution)


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

def get_index(x, y):
    return y * int(grid_dim_y) + x


def get_odom_pose(odom_msg):
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    yaw = normalize_angle(get_rotation(odom_msg))

    return np.array([x, y, yaw])
# ----------------------------------------------------------------------------------------------------------

def initialize_path_msg(curr_pose):
    path = Path()
    path.header.frame_id = "odom"
    
    pose = PoseStamped()
    pose.pose.position.x = curr_pose[0]
    pose.pose.position.y = curr_pose[1]
    pose.pose.position.z = 0
    

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2])
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    path.poses.append(pose)
    
    return path


def update_path_msg(path, curr_pose):

    pose = PoseStamped()
    pose.pose.position.x = curr_pose[0]
    pose.pose.position.y = curr_pose[1] 
    pose.pose.position.z = 0
    
    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2])
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    path.poses.append(pose)

    return path


def initialize_map_msg():
    map = OccupancyGrid()
    map.header.frame_id = "odom"
    map.info.resolution = map_resolution
    map.info.width  = grid_dim_x
    map.info.height = grid_dim_y
    map.info.origin.position.x = 0
    map.info.origin.position.y = 0
    map.info.origin.position.z = 0
    map.info.origin.orientation.x = 0
    map.info.origin.orientation.y = 0
    map.info.origin.orientation.z = 0    
    map.info.origin.orientation.w = 1
    
    # Add initial "Unknown values" to every cell:
    for i in range(grid_dim_x):
        for j in range(grid_dim_y):
            map.data.append(50)
    
    return map


def build_map(scan_msg, pose, map):
    angle_min = scan_msg.angle_min
    angle_incre = scan_msg.angle_increment
    ranges = scan_msg.ranges

    theta = pose[2]
    x_pos_t = int(pose[0] / map_resolution)
    y_pos_t = int(pose[1] / map_resolution)

    for i in range((len(ranges))):
        angle = angle_min + i * angle_incre

        z = ranges[i]
        if z == float('inf'):
            continue
   
        z_x = int(math.cos(angle + theta) * (z / map_resolution))
        z_y = int(math.sin(angle + theta) * (z / map_resolution))

        z_x_t = z_x + x_pos_t
        z_y_t = z_y + y_pos_t
        z_index = get_index(z_x_t, z_y_t)
        map.data[z_index] = 100
        
    return map


def initialize_particle(curr_pose):
    particles_msg = PoseArray()
    particles_msg.header.frame_id = "odom"
    particles_msg.header.stamp = rospy.Time.now()

    pose = Pose()
    pose.position.x = curr_pose[0]
    pose.position.y = curr_pose[1]
    pose.position.z = 0

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2])
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    particles_msg.poses.append(pose)

    return particles_msg


def update_particle(particles_msg, curr_pose):
    pose = Pose()
    pose.position.x = curr_pose[0]
    pose.position.y = curr_pose[1]
    pose.position.z = 0

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2])
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    particles_msg.poses.append(pose)

    return particles_msg



def update_map(icp_scan, curr_pose, map_msg):
    x = int(curr_pose[0] / map_resolution)
    y = int(curr_pose[1] / map_resolution)

    for i in range(len(icp_scan)):
        icp_x = icp_scan[i][0]
        icp_y = icp_scan[i][1]
        icp_index = get_index(icp_x, icp_y)

        map_msg.data[int(icp_index)] = 100
    
    return map_msg


if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("slam", anonymous=True)
    # Define ROS rate
    rate = rospy.Rate(1/dt)

    #initialize Graph
    graph_build.init_graph()

    # keep track of odom data
    ground_poses = []
    noisy_poses = []

    # get an odom message
    odom_msg =  rospy.wait_for_message("/odom", Odometry, timeout=None)
    init_odom = get_odom_pose(odom_msg)

    prev_odom = deepcopy(init_odom)

    # initial pose
    mark_pose = np.array([map_x/2, map_y/2, 0])

    ground_poses.append(mark_pose)
    noisy_poses.append(mark_pose)



    scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
    noisy_scan_msg = Noisy_sensor(scan_msg)

    vi = graph_build.create_vertex(noisy_scan_msg, mark_pose)

    # an initial node for loop detection created
    loop_closure = Loop_closure(vi.x_y_data)  

    path = initialize_path_msg(mark_pose)
    map = initialize_map_msg()
    particles = initialize_particle(mark_pose)

    path_pub = rospy.Publisher("/path", Path, queue_size=10)
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    particle_pub = rospy.Publisher("/particles", PoseArray, queue_size=1)

    map_msg = build_map(vi.scan_data, mark_pose, map)
    



    while not rospy.is_shutdown():
        # Measurements (odom) : getting noisy dx, dy and dyaw
        odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=None)
        curr_odom = get_odom_pose(odom_msg)
        dx_p = curr_odom[0] - prev_odom[0]
        dy_p = curr_odom[1] - prev_odom[1]
        dyaw_p = curr_odom[2] - prev_odom[2]
        r = math.sqrt(dx_p**2 + dy_p**2)
        
        if r > 0.2 or abs(dyaw_p) > 0.4:
           
            # Measurements (laser)
            scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
            noisy_scan_msg = Noisy_sensor(scan_msg)
            dx, dy, dyaw = noisy_odom.get_noisy_odom_2(prev_odom, curr_odom)

            prev_odom = deepcopy(curr_odom)
            mark_pose = np.array([mark_pose[0] + dx, mark_pose[1] + dy, mark_pose[2] + dyaw])

            ground_x = curr_odom[0] + int(-init_odom[0]) + int(map_x/2) 
            ground_y = curr_odom[1] + int(-init_odom[1]) + int(map_y/2)
            ground_yaw = curr_odom[2]
            ground_poses.append([ground_x, ground_y, ground_yaw])
            noisy_poses.append([mark_pose[0], mark_pose[1], mark_pose[2]])

            particles = update_particle(copy(particles), mark_pose)

            graph = graph_build.build_graph(noisy_scan_msg, mark_pose)
            vj = graph.verticies[-1]    
            map_msg = update_map(vj.x_y_data, vj.pose, deepcopy(map_msg))
      
            #plot(A, B, B_trans)
            #map_pub.publish(map_msg)

            if loop_closure.detect_loop(vj.x_y_data):
                uij = np.array([0, 0, 0])
                print("initial pose", graph.verticies[0].pose)
                edge = Edge(graph.verticies[0], vj, uij)
                graph.add_edges(edge)
                X = optimize_graph(graph)
                ground_posesss = np.array(ground_poses)
                noisy_pose = np.array(noisy_poses)
                plot_path(ground_poses, noisy_pose, X)

                
            
                break

        
        
        path_msg = update_path_msg(copy(path), mark_pose)
        path_pub.publish(path_msg)
        particle_pub.publish(particles)
        map_pub.publish(map_msg)



        rate.sleep()