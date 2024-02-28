#include <ros/ros.h>
#include <random>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf2_ros/transform_listener.h>          // Updated to use tf2_ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Include for transformations

// Define the sensor reading class
/*
    This library provides two ways to define a noisy camera model:
    - First, by providing the conventional noise covariance matrix (6x6) and the sensor frame name,
    - Second, by providing the sensor name and the covariance matrix (any shape) is allowed to be set later.

    The sensor pose is obtained from the tf tree;
    The sensor readings (relative poses of the target objects to the sensor) are obtained from the tf tree;
    Covariance matrix is set by the user, and should be determined by the sensor noise model.

    The sensor pose and the sensor readings are encapsulated in the geometry_msgs::PoseWithCovariance message
    by the customized covariance matrix.

    The sensor readings are published to the ROS topics, where the number of the topics is dynamic and determined by the number of the target objects.
    The published topics are named as "[OBJECT NAME]_noisy" for each target object. And the topics are indexed by the object numbering id.

    Note:
    - The published poses for both the sensor and the target objects are pure poses without noise added.
    - The noises are separately stored in the covariance matrix of the geometry_msgs::PoseWithCovariance messages


*/

class SensorNoisyReading
{
public:
    SensorNoisyReading(const std::string &sensor_frame_name,
                       const std::vector<std::string> &child_frames,
                       ros::NodeHandle &nh_,
                       const double *noise_std_distr[36]);
    // Constructor to initialize the sensor noisy reading
    /* noise_std_distr is recommended to have 36 elements for 6D pose
       But it could be 3x3 or 7x7 for position and (position, orientation) respectively */
    SensorNoisyReading(const std::string &sensor_frame_name,
                       const std::vector<std::string> &child_frames,
                       ros::NodeHandle &nh_);
    ~SensorNoisyReading();

    void setSensorNoise(const double *noise_std_distr, const int &dim); // Set the sensor noise covariance

    void updateSensorReading();
    void updateSensorState();                          // Set the sensor state, including the pose and its covariance
    void updateObjectState(size_t frame_numbering_id); // Set the object state, including the pose and its covariance

    /* Sensor information */
public:
    geometry_msgs::PoseWithCovarianceStamped sensor_state; // The camera pose and its covariance
    std::string sensor_frame_name;

private:
    std::vector<std::vector<double>> noise_covariance;
    size_t dimension; // Size of the noise covariance matrix, 36 for 6D pose

    /*Child frame information*/
public:
    std::vector<std::string> child_frames;
    geometry_msgs::PoseWithCovarianceStamped object_state; // The object pose and its covariance

    /* ROS members */
private:
    // Private members
    ros::NodeHandle nh;
    ros::Publisher sensor_state_pub;               // publisher that publishes the sensor state
    std::vector<ros::Publisher> object_state_pubs; // publishers that publish the noisy sensor readings

    tf2_ros::Buffer tfBuffer;                        // Use tf2_ros Buffer
    tf2_ros::TransformListener tfListener{tfBuffer}; // tf2 TransformListener
    geometry_msgs::TransformStamped sensor_tf_stamped_transform;
    geometry_msgs::TransformStamped object_tf_stamped_transform;
};
