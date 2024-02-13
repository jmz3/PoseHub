#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <posehub_tools/random_motion_engine.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_tf_broadcaster",
            ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  // Motion frequency
  double frequency;

  nh.param("frequency", frequency, 50.0);

  std::string name;
  nh.param<std::string>("frame_name", name, "default_frame_name");

  ObjectRandomMotionEngine object_motion(name, 1 / frequency, 2);
  object_motion.start();

  ros::Rate rate(frequency);

  while (ros::ok()) {
    object_motion.update();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = name;
    transformStamped.transform.translation.x =
        object_motion.object.pose.position.x;
    transformStamped.transform.translation.y =
        object_motion.object.pose.position.y;
    transformStamped.transform.translation.z =
        object_motion.object.pose.position.z;

    transformStamped.transform.rotation = object_motion.object.pose.orientation;

    br.sendTransform(transformStamped);

    rate.sleep();
  }

  return 0;
}
