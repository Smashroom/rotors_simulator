
#include <thread>
#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>

// Class for queueing the poses
class PoseQueue
{
public:
  PoseQueue(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &PoseQueue::poseCB, this);
    pose_queue_ready = false;
    pose_queue_ptr_ = new std::vector<Eigen::Vector3d> (5);
  }
  Eigen::Vector3d GrabLatestXYZ(){
    return pose_queue_ptr_->back();
  }
  bool pose_queue_ready;
private:
  void poseCB(const geometry_msgs::Pose& msg)
  {
    // Check whether pose queue it is on its limit
    if (pose_queue_ptr_->size() == pose_queue_ptr_->max_size()){
      pose_queue_ptr_->erase(pose_queue_ptr_->begin());
    }
    Eigen::Vector3d pose_msg_to_eigen_vector(msg.position.x, msg.position.y, msg.position.z);
    pose_queue_ptr_->push_back(pose_msg_to_eigen_vector);
    if(!pose_queue_ready){
      ROS_INFO("Pose queue is ready");
      pose_queue_ready = true;
    } 
  }
  ros::Subscriber sub_;
  std::vector<Eigen::Vector3d> *pose_queue_ptr_;
};

// Utilities
template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N-1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

// Trajectory Generator
std::vector<Eigen::Vector3d> take_off_waypoints_generator(const int smoothness, const int take_off_altitude, const Eigen::Vector3d initial_pose)
{
  std::vector<Eigen::Vector3d> result;

  const auto smoothening_constants = linspace<double>(0.1, 1, smoothness);
  Eigen::Vector3d waypoint_candidate(0.0, 0.0, take_off_altitude);

  for (auto &smoothening_constant : smoothening_constants)
  {
    result.push_back(initial_pose + waypoint_candidate * smoothening_constant);
  }
  return result;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_mapping");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "/firefly/command/trajectory", 10);
  
  PoseQueue pose_queue(nh, "/firefly/odometry_sensor1/pose");
  ros::AsyncSpinner spinner(8);
  spinner.start();
  //"mav_msgs::default_topics::COMMAND_TRAJECTORY
  ROS_INFO("Started mock mapping example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();
  
  // First find the current pose
  // Holding call until the global position is known
  while(true){
    ros::Duration(1.0).sleep();
    if(pose_queue.pose_queue_ready){
      ROS_INFO("Robot localised\n");
      break;
    }
  }

  // 2. take-off
  ROS_INFO("Progressive takeoff in action...");

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  const Eigen::Vector3d initial_pose = pose_queue.GrabLatestXYZ();

  // Default desired position and yaw.
  std::vector<Eigen::Vector3d> take_off_waypoints = take_off_waypoints_generator(3, 10, initial_pose);
  double desired_yaw = 0.0;
  for (std::vector<Eigen::Vector3d>::iterator it = take_off_waypoints.begin(); it != take_off_waypoints.end(); it++)
  {
    // Overwrite defaults if set as node parameters.
    nh_private.param("x", (*it)(0), (*it)(0));
    nh_private.param("y", (*it)(1), (*it)(1));
    nh_private.param("z", (*it)(2), (*it)(2));
    nh_private.param("yaw", desired_yaw, desired_yaw);
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        *it, desired_yaw, &trajectory_msg);

    ROS_INFO("Take-off waypoint on namespace %s: [%f, %f, %f].",
             nh.getNamespace().c_str(), (*it)(0),
             (*it)(1), (*it)(2));

    trajectory_pub.publish(trajectory_msg);
    ros::Duration(2.0).sleep();
  }

  //ros::spinOnce();
  
  // Default desired position and yaw.
  std::vector<Eigen::Vector3d> desired_positions;
  desired_positions.push_back(initial_pose + Eigen::Vector3d(-7.44644,  9.3534 , 13.728));
  desired_positions.push_back(initial_pose + Eigen::Vector3d(-5.94644, 10.8534 , 13.728));
  desired_positions.push_back(initial_pose + Eigen::Vector3d(-7.44644, 12.3534 , 13.728));
  desired_positions.push_back(initial_pose + Eigen::Vector3d(-8.94644, 10.8534 , 13.728));
  
  // Overwrite defaults if set as node parameters.
  for(auto &desired_position: desired_positions){
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    nh_private.param("x", desired_position.x(), desired_position.x());
    nh_private.param("y", desired_position.y(), desired_position.y());
    nh_private.param("z", desired_position.z(), desired_position.z());
    nh_private.param("yaw", desired_yaw, desired_yaw);

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, desired_yaw, &trajectory_msg);

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
            nh.getNamespace().c_str(), desired_position.x(),
            desired_position.y(), desired_position.z());
    trajectory_pub.publish(trajectory_msg);
    //std::this_thread::sleep_for(std::chrono::seconds(10));
   // ros::spinOnce();
    ros::Duration(10.0).sleep();


  }
  ros::shutdown();

  return 0;
}
