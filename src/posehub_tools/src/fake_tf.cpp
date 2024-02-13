#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <posehub_tools/random_motion_engine.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_tf_broadcaster");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Motion frequency
    double frequency = 10.0;

    ObjectRandomMotionEngine sensor_1("sensor_1", 1 / frequency);
    sensor_1.start();

    ros::Rate rate(frequency);

    while (ros::ok())
    {
        sensor_1.update();

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "sensor_1";
        transformStamped.transform.translation.x = sensor_1.object.pose.position.x;
        transformStamped.transform.translation.y = sensor_1.object.pose.position.y;
        transformStamped.transform.translation.z = sensor_1.object.pose.position.z;
        transformStamped.transform.rotation.x = sensor_1.object.pose.orientation.x;
        transformStamped.transform.rotation.y = sensor_1.object.pose.orientation.y;
        transformStamped.transform.rotation.z = sensor_1.object.pose.orientation.z;
        transformStamped.transform.rotation.w = sensor_1.object.pose.orientation.w;

        br.sendTransform(transformStamped);

        rate.sleep();
    }

    return 0;
}
