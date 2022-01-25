#!/usr/bin/env python

import rospy
import sys
import tf
import numpy as np

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform

from geometry_msgs.msg import Pose

pose_queue = []
pose_queue_size = 10
pose_queue_is_ready = False

def publish_waypoint(trajectory_pub, x_des,y_des,z_des, yaw_des):
    """
    Publish a waypoint to 
    """
    # create trajectory msg
    traj = MultiDOFJointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = 'frame'
    traj.joint_names.append('base_link')

    # create start point for trajectory
    
    transforms = Transform()
    velocities = Twist()
    accel = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(1))
    traj.points.append(point)
    
    #steps = 2
    #for idx_point in range(0,steps):
        # create end point for trajectory
    transforms = Transform()
    transforms.translation.x = x_des
    transforms.translation.y = y_des
    transforms.translation.z = z_des

    # Convert to quaternions
    quat = tf.transformations.quaternion_from_euler(yaw_des*np.pi/180.0, 0, 0, axes = 'rzyx')
    transforms.rotation.x = quat[0]
    transforms.rotation.y = quat[1]
    transforms.rotation.z = quat[2]
    transforms.rotation.w = quat[3]

    velocities = Twist()
    accel = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
    traj.points.append(point)

    trajectory_pub.publish(traj)
    rospy.sleep(1)


def clbk_odom(data):
    global pose_queue_is_ready
    # Check whether pose_queue is filled up
    if(len(pose_queue) >= pose_queue_size):
        pose_queue.pop(0)
    #print(f"Data is available, pose_queue size: {len(pose_queue)}" )
    pose_queue.append(data)
    if pose_queue_is_ready is False:
        pose_queue_is_ready = True

def GrabLastAvailablePose() -> Pose: # -> geometry_msgs/Pose:
    if len(pose_queue) < pose_queue_size:
        return pose_queue[len(pose_queue)]
    else:
        return pose_queue[pose_queue_size - 1]

def ConvertPoseToNpArr(pose_msg) -> np.ndarray:
    # Cartesian coordinate
    return np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])

def take_off(smoothness) -> np.ndarray:
    # Generate healthy take off waypoints
    return np.reshape(np.linspace(0,1,smoothness),(smoothness,1))

if __name__ == '__main__':
    try:
        rospy.init_node("mock_mapping", anonymous = True)
        trajectory_publisher = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)
        odometry_subscriber = rospy.Subscriber('/firefly/odometry_sensor1/pose', Pose, clbk_odom)
        print("start brooo")
        # When we will be working in robot_frame the initial starting point will be the origin
        # initial_coordinates=[]
        # All the movements are relative to the starting point
        # Generation of those are still vague - it needs to be automated

        take_off_waypoints = take_off(10)*np.reshape(np.array([0.0,0.0,13.728]),(1,3))
        
        #mapping_waypoints_relative_pose = np.reshape(np.array([[-7.44644,  9.3534 , 13.728],[-5.94644, 10.8534 , 13.728 ],[-7.44644, 12.3534 , 13.728],[-8.94644, 10.8534 , 13.728]]),(4,3))
            
        # Holding call until the position is known
        while(True):
            try:
                rospy.sleep(1)
                rospy.loginfo("Waitng the pose queue to be initialised")
                #print("Still waiting /debug_message")
                if(pose_queue_is_ready):
                    break
            except KeyboardInterrupt:
                break

        print("It is ready")
        rospy.loginfo("Pose queue has been initialised, it is time get last available pose")
        # Grab the initial_pose
        initial_pose = ConvertPoseToNpArr(GrabLastAvailablePose())
        
        rospy.loginfo("Latest available pose has been grabbed:\n {} \n calculated the waypoints in global coordinates".format(initial_pose))
        #mapping_waypoints_global_pose = np.concatenate([take_off_waypoints,mapping_waypoints_relative_pose]) + initial_pose
        mapping_waypoints_global_pose = take_off_waypoints
        # mapping_waypoints_relative_pose + initial_pose #

        # Traverse through each waypoint and wait for 10 seconds
        for waypoint in mapping_waypoints_global_pose:
            # Publish the mapping waypoint 
            publish_waypoint(trajectory_publisher, waypoint[0], waypoint[1], waypoint[2], 3.1)
            rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(waypoint[0], waypoint[1], waypoint[2], 3.1))
            rospy.sleep(10)

    except rospy.ROSInterruptException:
        print("ROS Terminated")
        pass
    except KeyboardInterrupt:
        sys.exit()
