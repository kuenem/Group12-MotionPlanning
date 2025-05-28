#include <cmath>
#include "quaternion_utils.h"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::msg::Quaternion getQuaternionFromXYZ(double x, double y, double z) {
    tf2::Quaternion q_x, q_y, q_z, q_combined;
    geometry_msgs::msg::Quaternion ros_quat;

    // Convert angles from degrees to radians
    double x_r = x * M_PI / 180.0;
    double y_r = y * M_PI / 180.0;
    double z_r = z * M_PI / 180.0;
    
    // Use tf2 quaternion for the conversion
    q_x.setRPY(x_r, 0, 0);
    q_y.setRPY(0, y_r, 0);
    q_z.setRPY(0, 0, z_r);

    q_combined = q_x * q_y * q_z;
    q_combined.normalize();

    // Convert to geometry_msgs::Quaternion
    ros_quat.w = q_combined.getW();
    ros_quat.x = q_combined.getX();
    ros_quat.y = q_combined.getY();
    ros_quat.z = q_combined.getZ();
    
    return ros_quat;
}