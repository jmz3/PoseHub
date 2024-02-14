
#include <posehub_tools/random_motion_engine.hpp>

ObjectInformation::ObjectInformation()
{
    // Initialize the object information
    object_name = "default";
}

ObjectInformation::~ObjectInformation()
{
    // Clear the object information
}

void ObjectInformation::setPose(double x, double y, double z, double qx,
                                double qy, double qz, double qw)
{
    // Set the object pose (position and orientation)
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
}

void ObjectInformation::setPose(tf2::Quaternion q)
{
    // Set the object orientation only if the input is a quaternion
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}

void ObjectInformation::setVelocity(double vx, double vy, double vz, double wx,
                                    double wy, double wz)
{
    // Set the object velocity
    velocity.linear.x = vx;
    velocity.linear.y = vy;
    velocity.linear.z = vz;
    velocity.angular.x = wx;
    velocity.angular.y = wy;
    velocity.angular.z = wz;
}

void ObjectInformation::setAcceleration(double ax, double ay, double az,
                                        double awx, double awy, double awz)
{
    // Set the object acceleration
    acceleration.linear.x = ax;
    acceleration.linear.y = ay;
    acceleration.linear.z = az;
    acceleration.angular.x = awx;
    acceleration.angular.y = awy;
    acceleration.angular.z = awz;
}

ObjectRandomMotionEngine::ObjectRandomMotionEngine(std::string const &name,
                                                   const double &T,
                                                   const double &MAX_P,
                                                   const double &MAX_V,
                                                   const double &MAX_A)
{
    // Initialize the object random motion engine
    object_state.object_name = name;

    // Set the maximum position, velocity, and acceleration
    max_position = MAX_P;
    max_velocity = MAX_V;
    max_acceleration = MAX_A;

    // Set the time step
    deltaT = T;

    // Set the random seed
    rng.seed(rd());
    urd = std::uniform_real_distribution<>(-1, 1);

    // Initialize the quaternion
    q = tf2::Quaternion(object_state.pose.orientation.x, object_state.pose.orientation.y,
                        object_state.pose.orientation.z, object_state.pose.orientation.w);
}

ObjectRandomMotionEngine::~ObjectRandomMotionEngine()
{
    // Deinitialize the random motion engine
    // ...
}

void ObjectRandomMotionEngine::start()
{
    // Initialize the motion parameters as zero
    object_state.setPose(0, 0, 0, 0, 0, 0, 1);
    object_state.setVelocity(0, 0, 0, 0, 0, 0);
    object_state.setAcceleration(0, 0, 0, 0, 0, 0);
    // Initialize the quaternion
    q = tf2::Quaternion(object_state.pose.orientation.x, object_state.pose.orientation.y,
                        object_state.pose.orientation.z, object_state.pose.orientation.w);
}

void ObjectRandomMotionEngine::update()
{
    // Update the random motion engine
    // Generate random acceleration
    object_state.acceleration.linear.x = ACC_SCALE * urd(rng);
    object_state.acceleration.linear.y = ACC_SCALE * urd(rng);
    object_state.acceleration.linear.z = ACC_SCALE * urd(rng);

    object_state.acceleration.angular.x = 10.0 * ACC_SCALE * urd(rng);
    object_state.acceleration.angular.y = 10.0 * ACC_SCALE * urd(rng);
    object_state.acceleration.angular.z = 10.0 * ACC_SCALE * urd(rng);

    // Sum the acceleration to the velocity
    object_state.velocity.linear.x += object_state.acceleration.linear.x * deltaT;
    object_state.velocity.linear.y += object_state.acceleration.linear.y * deltaT;
    object_state.velocity.linear.z += object_state.acceleration.linear.z * deltaT;

    object_state.velocity.angular.x += object_state.acceleration.angular.x * deltaT;
    object_state.velocity.angular.y += object_state.acceleration.angular.y * deltaT;
    object_state.velocity.angular.z += object_state.acceleration.angular.z * deltaT;

    // Sum the velocity to the position
    object_state.pose.position.x += object_state.velocity.linear.x * deltaT;
    object_state.pose.position.y += object_state.velocity.linear.y * deltaT;
    object_state.pose.position.z += object_state.velocity.linear.z * deltaT;

    // Sum the angular velocity to the orientation
    // This is a "fake" implementation, it should be replaced by a proper
    // quaternion integration This is a pure quaternion with no scalar part
    q += tf2::Quaternion(object_state.velocity.angular.x * deltaT,
                         object_state.velocity.angular.y * deltaT,
                         object_state.velocity.angular.z * deltaT, 0);
    q.normalize();

    // Set the new orientation
    object_state.setPose(q);

    // Post-processing to enforce the limits
    // Check the position limits
    if (abs(object_state.pose.position.x) > max_position)
    {
        // If the position is out of the limits, set it to the limit and invert the
        // velocity
        object_state.pose.position.x =
            (object_state.pose.position.x > 0) ? max_position : -max_position;
        object_state.velocity.linear.x = -object_state.velocity.linear.x;
    }
    else if (abs(object_state.pose.position.y) > max_position)
    {
        object_state.pose.position.y =
            (object_state.pose.position.y > 0) ? max_position : -max_position;
        object_state.velocity.linear.y = -object_state.velocity.linear.y;
    }
    else if (abs(object_state.pose.position.z) > max_position)
    {
        object_state.pose.position.z =
            (object_state.pose.position.z > 0) ? max_position : -max_position;
        object_state.velocity.linear.z = -object_state.velocity.linear.z;
    }

    // Check the velocity limits
    if (abs(object_state.velocity.linear.x) > max_velocity)
    {
        object_state.velocity.linear.x = object_state.velocity.linear.x / 3; // max_velocity or zero better?
    }
    else if (abs(object_state.velocity.linear.y) > max_velocity)
    {
        object_state.velocity.linear.y = object_state.velocity.linear.y / 3;
    }
    else if (abs(object_state.velocity.linear.z) > max_velocity)
    {
        object_state.velocity.linear.z = object_state.velocity.linear.z / 3;
    }
}

void ObjectRandomMotionEngine::stop()
{
    // Stop the random motion engine
}

ObjectStaticEngine::ObjectStaticEngine(std::string const &name,
                                       geometry_msgs::PosePtr &pose,
                                       const double &T)
{
    // Initialize the object static engine
    object_state.object_name = name;

    // Set the object pose using the input argument
    object_state.pose.position = pose->position;
    object_state.pose.orientation = pose->orientation;

    // Set the object velocity and acceleration as zero since the object is static
    object_state.setVelocity(0, 0, 0, 0, 0, 0);
    object_state.setAcceleration(0, 0, 0, 0, 0, 0);

    // Set the time step
    deltaT = T;
}

ObjectStaticEngine::ObjectStaticEngine(std::string const &name, const double &T)
{
    // Initialize the object static engine
    object_state.object_name = name;

    // Set the object pose at the origin
    object_state.setPose(0, 0, 0, 0, 0, 0, 1);
    object_state.setVelocity(0, 0, 0, 0, 0, 0);
    object_state.setAcceleration(0, 0, 0, 0, 0, 0);

    // Set the time step
    deltaT = T;
}

ObjectStaticEngine::~ObjectStaticEngine()
{
    // Deinitialize the static engine
    object_state.~ObjectInformation();
}

void ObjectStaticEngine::start()
{
    // Initialize the static tf frame for the object
}

void ObjectStaticEngine::update(geometry_msgs::PosePtr &pose)
{
    // change the object pose if needed
    object_state.pose.position = pose->position;
    object_state.pose.orientation = pose->orientation;
}
