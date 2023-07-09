#include <ros/ros.h>
#include "ros_rest_interface/ros_message_gateway.h"
#include "ros_rest_interface/point_msgs.h"

using namespace geometry_msgs;

namespace ros_rest_interface
{
    RosMessageGateway::RosMessageGateway(ros::NodeHandle &nh, const std::vector<std::string> &default_topics) : nh_(nh)
    {
        for (std::string topic : default_topics)
        {
            initSubscribe(topic);
        }
    }

    void RosMessageGateway::sendMessage(const std::string &topic, Point msg)
    {
        if (!publishers_.contains(topic))
        {
            initPublisher(topic);
        }

        auto pub = publishers_.get(topic);
        // avoid computational overhead for topics which no nodes are subscribed to.
        if (pub.getNumSubscribers() > 0)
            pub.publish(msg);
    };

    Point RosMessageGateway::getLastMessage(const std::string &topic)
    {

        if (!subscribers_.contains(topic))
        {
            ROS_INFO_STREAM("init subscriber - " << topic << "total" << subscribers_.size() << std::endl);
            initSubscribe(topic);
        }

        return last_messages_.get(topic);
    };

    void RosMessageGateway::initSubscribe(const std::string &topic)
    {

        auto updateLastMessage = [this, topic](const Point::ConstPtr &msg)
        {
            last_messages_.insert(topic, *msg);
        };
        ros::Subscriber sub = nh_.subscribe<Point>(topic, 1000, updateLastMessage);
        subscribers_.insert(topic, sub);
        last_messages_.insert(topic, NotExistsPoint());
    }

    void RosMessageGateway::initPublisher(const std::string &topic)
    {
        ros::Publisher pub = nh_.advertise<Point>(topic, 100, true);
        publishers_.insert(topic, pub);
    }
}
