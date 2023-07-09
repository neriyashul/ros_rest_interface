#include "ros_rest_interface/web_server.h"
#include "ros_rest_interface/web_error.h"
#include "ros_rest_interface/controller_interface.h"

using namespace web::http;
using namespace web::json;

namespace web_http
{
    WebServer::WebServer(web::uri address, web_http::ControllerInterface& contoller) : listener_(address), controller_(contoller)
    {
        listener_.support(methods::GET, std::bind(&WebServer::handleGetRequest, this, std::placeholders::_1));
        listener_.support(methods::POST, std::bind(&WebServer::handlePostRequest, this, std::placeholders::_1));
    }

    void WebServer::start()
    {
        try
        {
            listener_.open().wait();
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Failed to start the web server: " + std::string(e.what()));
        }
    }

    void WebServer::close()
    {
        try
        {
            listener_.close().wait();
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Failed to close the web server: " + std::string(e.what()));
        }
    }

    void WebServer::handleGetRequest(const http_request& request)
    {
        std::string path = WebServer::getRequestPath(request);
        web::json::value response = controller_.handleGet(path);
        request.reply(status_codes::OK, response);
    }

    http_response createRespone(const status_code status, std::string msg)
    {
        web::json::value response;
        http_response res(status);
        res.set_body(msg);
        return res;
    }

    void WebServer::handlePostRequest(const http_request& request)
    {
        request.extract_json()
            .then([request, this](pplx::task<web::json::value> task)
                  {
            const web::json::value& data = task.get();
            try
            {
                controller_.handlePost(data);
                web::json::value response = web::json::value::string(U("success"));
                http_response res(status_codes::OK);
                res.headers().add("Content-Type", "application/json");
                res.set_body(response);

                request.reply(res);
            }
            catch (const web_http::BadRequestError& e)
            {
                http_response res = createRespone(status_codes::BadRequest, e.what());
                request.reply(res);
            }
            catch (const web_http::InternalError& e)
            {
                http_response res = createRespone(status_codes::InternalError, e.what());
                request.reply(res);
            }
            catch (const std::exception& e)
            {
                auto msg = "Unknown error: " + std::string(e.what());
                http_response res = createRespone(status_codes::InternalError, msg);
                request.reply(res);
            } });
    }

    std::string WebServer::getRequestPath(const http_request& request)
    {
        const web::uri& uri = request.request_uri();
        std::string path = utility::conversions::to_utf8string(uri.path());
        return path;
    }
}