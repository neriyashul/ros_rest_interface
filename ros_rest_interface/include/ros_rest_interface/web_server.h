

#pragma once

#include <cpprest/json.h>
#include <cpprest/http_listener.h>
#include <cpprest/uri.h>
#include <cpprest/asyncrt_utils.h>
#include "ros_rest_interface/controller_interface.h"

namespace web_http
{
    /*!
    * The class responsible for handling HTTP requests 
    * and interacting with a ControllerInterface.
    */
    class WebServer
    {
    public:
        /*!
        * Constructor
        * @param address   The web::uri representing the address to bind the server.
        * @param controller The reference to a ControllerInterface for request handling.
        */
        WebServer(web::uri address, web_http::ControllerInterface& controller);

        /*!
        * Destructor.
        */
        ~WebServer(){};

        /*!
        * Starts the web server and begins listening for incoming requests.
        */
        void start();

        /*!
        * Closes the web server, stopping it from accepting new requests.
        */
        void close();

        /*!
        * Handles an incoming HTTP GET request.
        * @param request The web::http::http_request object representing the received request.
        */
        void handleGetRequest(const web::http::http_request& request);

        /*!
        * Handles an incoming HTTP POST request.
        * @param request The web::http::http_request object representing the received request.
        */
        void handlePostRequest(const web::http::http_request& request);

    private:
        /*!
        * Retrieves the request path from the provided HTTP request object.
        * @param request The web::http::http_request object representing the received request.
        * @return        The path of the request as a string.
        */
        std::string getRequestPath(const web::http::http_request& request);

        web::http::experimental::listener::http_listener listener_;     // The HTTP listener object for handling requests.
        ControllerInterface& controller_;                                // The reference to a ControllerInterface for request handling.
    };


}