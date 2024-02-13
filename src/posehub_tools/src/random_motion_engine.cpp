
#include <posehub_tools/random_motion_engine.hpp>

ObjectInformation::ObjectInformation() {
  // Initialize the object information
  object_name = "default";
}

ObjectInformation::~ObjectInformation() {
  // Clear the object information
}

void ObjectInformation::setPose(double x, double y, double z, double qx,
                                double qy, double qz, double qw) {
  // Set the object pose (position and orientation)
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
}

void ObjectInformation::setPose(tf2::Quaternion q) {
  // Set the object orientation only if the input is a quaternion
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
}

void ObjectInformation::setVelocity(double vx, double vy, double vz, double wx,
                                    double wy, double wz) {
  // Set the object velocity
  velocity.linear.x = vx;
  velocity.linear.y = vy;
  velocity.linear.z = vz;
  velocity.angular.x = wx;
  velocity.angular.y = wy;
  velocity.angular.z = wz;
}

void ObjectInformation::setAcceleration(double ax, double ay, double az,
                                        double awx, double awy, double awz) {
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
                                                   const double &MAX_A) {
  // Initialize the object random motion engine
  object.object_name = name;

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
  q = tf2::Quaternion(object.pose.orientation.x, object.pose.orientation.y,
                      object.pose.orientation.z, object.pose.orientation.w);
}

ObjectRandomMotionEngine::~ObjectRandomMotionEngine() {
  // Deinitialize the random motion engine
  // ...
}

void ObjectRandomMotionEngine::start() {
  // Initialize the motion parameters as zero
  object.setPose(0, 0, 0, 0, 0, 0, 1);
  object.setVelocity(0, 0, 0, 0, 0, 0);
  object.setAcceleration(0, 0, 0, 0, 0, 0);
  // Initialize the quaternion
  q = tf2::Quaternion(object.pose.orientation.x, object.pose.orientation.y,
                      object.pose.orientation.z, object.pose.orientation.w);
}

void ObjectRandomMotionEngine::update() {
  // Update the random motion engine
  // Generate random acceleration
  object.acceleration.linear.x = ACC_SCALE * urd(rng);
  object.acceleration.linear.y = ACC_SCALE * urd(rng);
  object.acceleration.linear.z = ACC_SCALE * urd(rng);

  object.acceleration.angular.x = 10.0 * ACC_SCALE * urd(rng);
  object.acceleration.angular.y = 10.0 * ACC_SCALE * urd(rng);
  object.acceleration.angular.z = 10.0 * ACC_SCALE * urd(rng);

  // Sum the acceleration to the velocity
  object.velocity.linear.x += object.acceleration.linear.x * deltaT;
  object.velocity.linear.y += object.acceleration.linear.y * deltaT;
  object.velocity.linear.z += object.acceleration.linear.z * deltaT;

  object.velocity.angular.x += object.acceleration.angular.x * deltaT;
  object.velocity.angular.y += object.acceleration.angular.y * deltaT;
  object.velocity.angular.z += object.acceleration.angular.z * deltaT;

  // Sum the velocity to the position
  object.pose.position.x += object.velocity.linear.x * deltaT;
  object.pose.position.y += object.velocity.linear.y * deltaT;
  object.pose.position.z += object.velocity.linear.z * deltaT;

  // Sum the angular velocity to the orientation
  // This is a "fake" implementation, it should be replaced by a proper
  // quaternion integration This is a pure quaternion with no scalar part
  q += tf2::Quaternion(object.velocity.angular.x * deltaT,
                       object.velocity.angular.y * deltaT,
                       object.velocity.angular.z * deltaT, 0);
  q.normalize();

  // Set the new orientation
  object.setPose(q);

  // Post-processing to enforce the limits
  // Check the position limits
  if (abs(object.pose.position.x) > max_position) {
    // If the position is out of the limits, set it to the limit and invert the
    // velocity
    object.pose.position.x =
        (object.pose.position.x > 0) ? max_position : -max_position;
    object.velocity.linear.x = -object.velocity.linear.x;
  } else if (abs(object.pose.position.y) > max_position) {
    object.pose.position.y =
        (object.pose.position.y > 0) ? max_position : -max_position;
    object.velocity.linear.y = -object.velocity.linear.y;
  } else if (abs(object.pose.position.z) > max_position) {
    object.pose.postion.z =
        (object.pose.position.z > 0) ? max_position : -max_position;
    object.velocity.linear.z = -object.velocity.linear.z;
  }

  // Check the velocity limits
  if (abs(object.velocity.linear.x) > max_velocity) {
    object.velocity.linear.x = max_velocity;  // or 0 ?
  } else if (abs(object.velocity.linear.y) > max_velocity) {
    object.velocity.linear.y = max_velocity;
  } else if (abs(object.velocity.linear.z) > max_velocity) {
    object.velocity.linear.z = max_velocity;
  }
}

void ObjectRandomMotionEngine::stop() {
  // Stop the random motion engine
}