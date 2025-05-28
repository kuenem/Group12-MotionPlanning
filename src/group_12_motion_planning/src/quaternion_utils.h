#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::msg::Quaternion getQuaternionFromXYZ(double x, double y, double z);

#endif
