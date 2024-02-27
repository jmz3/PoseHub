/* This is a ROS node that creates a fake sensor noise for the input poses.
It subscribes to all the topics that match the specified pattern, which is "object" by default,
 and adds noise to the input poses to mimic the real sensor readings. */

#include <ros/ros.h>
#include <ros/master.h>
#include <vector>
#include <string>
#include <regex>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

// Function to get the list of topics that match the input pattern using regular expressions
std::vector<std::string> getTopicsWithPattern(const std::string &pattern)
{
    std::vector<ros::master::TopicInfo> all_topics;
    std::vector<std::string> filtered_topics;

    // Get the list of all topics
    ros::master::getTopics(all_topics);

    // Compile the regular expression
    std::regex regex_pattern(".*" + pattern + ".*");

    // Filter topics that match the input pattern
    for (const auto &topic_info : all_topics)
    {
        try
        {
            if (std::regex_match(topic_info.name, regex_pattern))
            {
                filtered_topics.push_back(topic_info.name);
            }
        }
        catch (std::regex_error &e)
        {
            ROS_ERROR("Regex error: %s", e.what());
            // Handle regex error (e.g., log, throw exception, etc.)
        }
    }

    return filtered_topics;
}

void fake_sensor_noise_callback(const geometry_msgs::PoseConstPtr &msg)
{
    // Add noise to the input pose
    geometry_msgs::Pose noised_pose = *msg;

    // Add
    // ...
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_sensor_noise_node");
    ros::NodeHandle nh;

    ros::Subscriber fake_sensor_noise_sub;
    ros::Publisher fake_sensor_noise_pub;

    std::vector<std::string> filtered_topics = getTopicsWithPattern("object");

    for (const auto &topic : filtered_topics)
    {
        ROS_INFO("Topic: %s", topic.c_str());
        fake_sensor_noise_sub = nh.subscribe(topic, 10, fake_sensor_noise_callback);
    }

    ros::Rate rate(100);

    // Get the list of topics that match the pattern

    // Print the list of filtered topics
    while (nh.ok())
    {

        rate.sleep();
    }

    return 0;
}
