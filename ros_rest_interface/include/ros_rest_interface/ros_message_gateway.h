#pragma once
#include <ros/ros.h>
#include <cpprest/json.h>
#include "ros_rest_interface/concurrent_unordered_map.h"
#include "geometry_msgs/Point.h"

namespace ros_rest_interface
{
    /* The class is responsible for facilitating communication 
     * with ROS topics through message sending and receiving operations.
     */
    class RosMessageGateway
    {
    public:
        /*!
        * Constructor for RosMessageGateway.
        * @param nh             The reference to ros::NodeHandle.
        * @param default_topics The vector of default topics.
        */
        RosMessageGateway(ros::NodeHandle& nh, const std::vector<std::string>& default_topics);

        /*!
        * Sends a message on the specified topic.
        * @param topic The topic to send the message on.
        * @param msg   The geometry_msgs::Point message to send.
        */
        void sendMessage(const std::string& topic, geometry_msgs::Point msg);

        /*!
        * Retrieves the last received message for the specified topic.
        * @param topic The topic to retrieve the last message from.
        * @return      The last received geometry_msgs::Point message.
        */
        geometry_msgs::Point getLastMessage(const std::string& topic);

    private:
        ros::NodeHandle& nh_;                                                 // The reference to ros::NodeHandle.
        concurrent_collection::ConcurrentUnorderedMap<std::string, ros::Publisher> publishers_;     // Map of topic publishers.
        concurrent_collection::ConcurrentUnorderedMap<std::string, ros::Subscriber> subscribers_;   // Map of topic subscribers.
        concurrent_collection::ConcurrentUnorderedMap<std::string, geometry_msgs::Point> last_messages_;  // Map of last received messages.

        /*!
        * Initializes a subscriber for the specified topic.
        * @param topic The topic to initialize the subscriber for.
        */
        void initSubscribe(const std::string& topic);

        /*!
        * Initializes a publisher for the specified topic.
        * @param topic The topic to initialize the publisher for.
        */
        void initPublisher(const std::string& topic);
    };

}