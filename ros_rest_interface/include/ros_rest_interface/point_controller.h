#pragma once
#include <ros/ros.h>
#include "ros_rest_interface/controller_interface.h"
#include "ros_rest_interface/ros_message_gateway.h"
#include "geometry_msgs/Point.h"

namespace ros_rest_interface
{
    /*!
     * The class acts as a mediator between HTTP requests and the data entity, 
     * handling the request response for point-related data.
     */
    class PointController : public web_http::ControllerInterface
    {
    public:
        /*!
        * Constructor
        * @param message_gateway The reference to RosMessageGateway.
        */
        PointController(ros_rest_interface::RosMessageGateway& message_gateway);

        /*!
        * Handles HTTP GET request.
        * @param data The data associated with the GET request.
        * @return     The response as a JSON value.
        */
        web::json::value handleGet(const std::string& data) override;

        /*!
        * Handles HTTP POST request.
        * @param data The data associated with the POST request.
        */
        void handlePost(const web::json::value& data) override;

    private:
        ros_rest_interface::RosMessageGateway& message_gateway_;  // The reference to RosMessageGateway.

        /*!
        * Converts JSON data to a geometry_msgs::Point.
        * @param data The JSON data to be converted.
        * @return     The converted geometry_msgs::Point.
        */
        geometry_msgs::Point jsonToPoint(const web::json::value& data);

        /*!
        * Converts a geometry_msgs::Point to JSON.
        * @param point The geometry_msgs::Point to be converted.
        * @return       The converted JSON value.
        */
        web::json::value pointToJson(const geometry_msgs::Point& point);
    };
}