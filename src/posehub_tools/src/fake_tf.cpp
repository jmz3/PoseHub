#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <posehub_tools/random_motion_engine.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_tf_broadcaster");
    ros::NodeHandle nh("~"); // Private node handle to get the private parameters

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Publisher frame_state_pub;

    // Motion frequency
    double frequency;

    nh.param("frequency", frequency, 100.0);

    std::string name;
    nh.param<std::string>("frame_name", name, "default_frame_name");

    ObjectRandomMotionEngine object_motion(name, 1 / frequency, 1.0, 0.5, 1.0);
    object_motion.start();

    ros::Rate rate(frequency);

    frame_state_pub = nh.advertise<geometry_msgs::Pose>(name, 10);
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = name;

    while (ros::ok())
    {
        object_motion.update();
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x =
            object_motion.object_state.pose.position.x;
        transformStamped.transform.translation.y =
            object_motion.object_state.pose.position.y;
        transformStamped.transform.translation.z =
            object_motion.object_state.pose.position.z;

        transformStamped.transform.rotation = object_motion.object_state.pose.orientation;

        br.sendTransform(transformStamped);

        frame_state_pub.publish(object_motion.object_state.pose);

        rate.sleep();
    }

    object_motion.stop();

    return 0;
}
