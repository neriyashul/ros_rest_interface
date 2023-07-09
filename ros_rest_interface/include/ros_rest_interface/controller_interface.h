#pragma once
#include <cpprest/json.h>

namespace web_http
{
    /*!
     * Controller Interface for handling HTTP requests.
     */
    class ControllerInterface
    {
    public:
        /*!
         * Handles HTTP GET request.
         * @param data The data associated with the GET request.
         * @return     The response as a JSON value.
         */
        virtual web::json::value handleGet(const std::string& data) = 0;

        /*!
         * Handles HTTP POST request.
         * @param data The data associated with the POST request.
         */
        virtual void handlePost(const web::json::value& data) = 0;
    };
}
