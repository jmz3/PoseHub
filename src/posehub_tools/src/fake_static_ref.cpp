#include <posehub_tools/random_motion_engine.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_static_ref");
    ros::NodeHandle nh("~"); // Private node handle to get the private parameters

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Publisher frame_state_pub;

    // Motion frequency
    double frequency;

    nh.param("frequency", frequency, 100.0);

    std::string name;
    nh.param<std::string>("frame_name", name, "default_static_frame_name");

    ROS_INFO("Correctly initialized the node");

    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.5;
    object_pose.position.z = 0.0;
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;

    // Convert Pose to PosePtr (shared pointer)
    geometry_msgs::PosePtr object_pose_ptr = boost::make_shared<geometry_msgs::Pose>(object_pose);

    ObjectStaticEngine object_static(name, object_pose_ptr, 1 / frequency);

    ros::Rate rate(frequency);

    frame_state_pub = nh.advertise<geometry_msgs::Pose>(name, 10);

    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = name;

    while (ros::ok())
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = object_static.object_state.pose.position.x;
        transformStamped.transform.translation.y = object_static.object_state.pose.position.y;
        transformStamped.transform.translation.z = object_static.object_state.pose.position.z;
        transformStamped.transform.rotation = object_static.object_state.pose.orientation;

        br.sendTransform(transformStamped);
        frame_state_pub.publish(object_static.object_state.pose);

        rate.sleep();
    }

    return 0;
}
