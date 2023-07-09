#pragma once
#include <geometry_msgs/Point.h>

namespace ros_rest_interface
{
    class NotExistsPoint : public geometry_msgs::Point
    {
    public:
        NotExistsPoint() {
            x = -std::numeric_limits<double>::infinity();
            y = -std::numeric_limits<double>::infinity();
            z = -std::numeric_limits<double>::infinity();
        }
    };
}