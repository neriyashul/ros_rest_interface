
#include <ros/ros.h>
#include "ros_rest_interface/point_controller.h"
#include "ros_rest_interface/web_error.h"

using namespace web;
using namespace web_http;
using namespace geometry_msgs;

namespace ros_rest_interface
{

    PointController::PointController(ros_rest_interface::RosMessageGateway& message_gateway) : message_gateway_(message_gateway)
    {
    }

    json::value PointController::handleGet(const std::string& topic)
    {
        ROS_INFO_STREAM("handle get: " << topic << std::endl);
        json::value response;
        
        Point point; 
        
        try {
            point = message_gateway_.getLastMessage(topic);
        } catch (const std::exception& )
        {
            ROS_ERROR_STREAM("Internal error while getting data from topic: " << topic);
            throw InternalError("Internal error while getting data from topic: " + topic);
        }
        
        try
        {
            response["topic"] = json::value::string(U(topic));
            response["data"] = pointToJson(point);
        }
        catch (const std::exception& )
        {
            ROS_ERROR("Invalid or missing values in 'data' object");
            throw BadRequestError("Invalid or missing values in 'data' object");
        }
        return response;
    }

    void PointController::handlePost(const json::value& json)
    {
        std::string topic;
        Point point;

        try
        {
            ROS_INFO_STREAM("handle post: " << json << std::endl);
            topic = json.at("topic").as_string();
            point = jsonToPoint(json.at("data"));
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Invalid or missing values in 'data' object.");
            throw BadRequestError("Invalid or missing values in 'data' object.");
        }

        try 
        {
            message_gateway_.sendMessage(topic, point);
        }
        catch (const std::exception& )
        {
            ROS_ERROR_STREAM("Internal error while sending data to topic: " << topic;);
            throw InternalError("Internal error while sending data to topic: " + topic);
        }
    }

    Point PointController::jsonToPoint(const json::value& data) {
        Point point;
        point.x = data.at("x").as_double();
        point.y = data.at("y").as_double();
        point.z = data.at("z").as_double();
        return point;
    }

    json::value PointController::pointToJson(const Point& point) {
        json::value jsonPoint;
        jsonPoint["x"] = json::value::number(point.x);
        jsonPoint["y"] = json::value::number(point.y);
        jsonPoint["z"] = json::value::number(point.z);
        return jsonPoint;
    }

}