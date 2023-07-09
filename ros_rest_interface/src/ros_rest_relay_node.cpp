#include <ros/ros.h>
#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <cpprest/uri.h>
#include <cpprest/asyncrt_utils.h>

#include "geometry_msgs/Point.h"

#include "ros_rest_interface/web_server.h"
#include "ros_rest_interface/point_controller.h"
#include "ros_rest_interface/ros_message_gateway.h"

const std::vector<std::string> getDefaultTopics(const ros::NodeHandle &nh)
{
    std::vector<std::string> default_topics;
    if (!nh.getParam("defualt_subscribed_topics", default_topics))
    {
        // no default topic
        default_topics = std::vector<std::string>{};
    }
    return default_topics;
}

int runServerNode(const std::string& uri, ros::NodeHandle& nh)
{
    std::vector<std::string> topics_to_subscribe = getDefaultTopics(nh);
    ros_rest_interface::RosMessageGateway gateway(nh, topics_to_subscribe);
    ros_rest_interface::PointController controller(gateway);

    web_http::WebServer server(uri, controller);
    server.start();
    ROS_INFO_STREAM("Web server started at: " << uri);
    ros::spin();
    server.close();
    ROS_INFO_STREAM("Web server closed" << uri);

    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "ros_rest_relay");
        ros::NodeHandle nh("~");
        ROS_INFO("node 'ros_rest_relay' started");

        std::string uri;
        if (!nh.getParam("uri", uri))
        {
            ROS_ERROR("There is no uri argument");
            ros::shutdown();
            return 1;
        }

        return runServerNode(uri, nh);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("An error occurred: " << e.what());
        ros::shutdown();
        return 1;
    }
}

