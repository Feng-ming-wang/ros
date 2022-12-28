#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <mutex>
#include <condition_variable>

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

bool is_first = true, is_first_yaw = true;
nav_msgs::Odometry last_rtk;
nav_msgs::Odometry odomReal;
nav_msgs::Path path;
nav_msgs::Path path_cal;
std::string gps_topic = "/pioneer_sensors/EKF_Localization_RS232/filteredodometry";
ros::Publisher pubRtk;
ros::Publisher path_pub;
ros::Publisher pathCal_pub;

double diff_y, diff_x, yaw_cal, last_yaw;

// void publish_odometry(const nav_msgs::Odometry::ConstPtr &msg, const ros::Publisher &pubRtk) {
    
// }

void rtk_cbk(const nav_msgs::Odometry::ConstPtr &msg_in) {
    nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry(*msg_in));
    mtx_buffer.lock();

    if (is_first) {
        last_rtk = *msg;
        is_first = false;
    }
    
    odomReal.header.frame_id = "world";
    odomReal.child_frame_id = "body";
    odomReal.header.stamp = ros::Time::now();
    odomReal.pose.pose.position.x = msg->pose.pose.position.x;
    odomReal.pose.pose.position.y = msg->pose.pose.position.y;
    odomReal.pose.pose.position.z = msg->pose.pose.position.z;
    odomReal.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odomReal.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odomReal.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odomReal.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomReal.header.stamp, "world", "body"));

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.pose.position.z;
    this_pose_stamped.pose.orientation = msg->pose.pose.orientation;
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    std::cout << "yaw: " << yaw << std::endl;
    
    double roll_cal = 0.0, pitch_cal = 0.0;
    diff_y = msg->pose.pose.position.y - last_rtk.pose.pose.position.y;
    diff_x = msg->pose.pose.position.x - last_rtk.pose.pose.position.x;
    yaw_cal = std::atan2(diff_y, diff_x);
    std::cout << "yaw_cal: " << yaw_cal << std::endl;
    if (is_first_yaw) {
        last_yaw = yaw_cal;
        is_first_yaw = false;
    }
    if ((yaw_cal - last_yaw) > 0.1) {
        yaw_cal = last_yaw + 0.1;
    }
    if ((yaw_cal - last_yaw) < -0.1) {
        yaw_cal = last_yaw - 0.1;
    }

    last_yaw = yaw_cal;
    // yaw_cal = 0.5;
    tf::Quaternion quater;
    geometry_msgs::Quaternion msg_q;
    // quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw_cal);
    quater.setRPY(roll_cal, pitch_cal, yaw_cal);
    tf::quaternionTFToMsg(quater, msg_q);
    geometry_msgs::PoseStamped pose_stamped_cal;
    pose_stamped_cal.pose.position.x = msg->pose.pose.position.x;
    pose_stamped_cal.pose.position.y = msg->pose.pose.position.y;
    pose_stamped_cal.pose.position.z = msg->pose.pose.position.z;
    pose_stamped_cal.pose.orientation = msg_q;
    pose_stamped_cal.header.stamp = ros::Time::now();
    pose_stamped_cal.header.frame_id = "world";

    path.poses.push_back(this_pose_stamped);
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    path_cal.poses.push_back(pose_stamped_cal);
    path_cal.header.stamp = ros::Time::now();
    path_cal.header.frame_id = "world";

    last_rtk = *msg;

    path_pub.publish(path);
    pathCal_pub.publish(path_cal);
    pubRtk.publish(odomReal);
    // std::cout << "Successfully publish odometry." << std::endl;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_rtk");
    ros::NodeHandle nh;
    ros::Subscriber sub_rtk = nh.subscribe(gps_topic, 200000, rtk_cbk);
    pubRtk = nh.advertise<nav_msgs::Odometry>("/real_rtk", 10);
    path_pub = nh.advertise<nav_msgs::Path>("trajectory_odom", 10, true);
    pathCal_pub = nh.advertise<nav_msgs::Path>("calculate_odom", 10, true);

    std::cout << "订阅RTK!!!" << std::endl;

    ros::Rate loop_rate(50);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}