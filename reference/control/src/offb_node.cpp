/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4
 * Pro Flight Stack and tested in Gazebo SITL
 */

 #include <geometry_msgs/PoseStamped.h>
 #include <ros/ros.h>
 #include <Eigen/Dense>
 #include <nav_msgs/Odometry.h>
 #include <Eigen/Eigen>

 Eigen::Vector3d pos_drone_t265; // odom位置
 Eigen::Quaterniond q_t265;      // odom位置四元数

 // odom位置
 void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
 {

     pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);

     q_t265 = Eigen::Quaterniond(
         msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
     // Euler_t265 = quaternion_to_euler(q_gazebo);
     //  Euler_t265[2] = Euler_t265[2] + yaw_offset;
     //  q_t265 = quaternion_from_rpy(Euler_t265);
 }

 int main(int argc, char **argv)
 {

     ros::init(argc, argv, "t265_to_mavros");
     ros::NodeHandle nh;

     //  【订阅】用t265估计位置
     ros::Subscriber t265_sub =
         nh.subscribe<nav_msgs::Odometry>("/Odometry_offset", 5, t265_cb);

     // 发布位置信息
     ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
         "/mavros/vision_pose/pose", 10);

     // the setpoint publishing rate MUST be faster than 2Hz
     ros::Rate rate(50.0);

     ros::Time last_request = ros::Time::now();
     ROS_INFO("pose tran have started!");
     while (ros::ok())
     {
         geometry_msgs::PoseStamped vision;

         vision.pose.position.x = pos_drone_t265[0];
         vision.pose.position.y = pos_drone_t265[1];
         vision.pose.position.z = pos_drone_t265[2];

         vision.pose.orientation.x = q_t265.x();
         vision.pose.orientation.y = q_t265.y();
         vision.pose.orientation.z = q_t265.z();
         vision.pose.orientation.w = q_t265.w();

         // 赋值时间戳
         vision.header.stamp = ros::Time::now();
         vision_pub.publish(vision);

         ros::spinOnce();
         rate.sleep();
     }

     return 0;
 }