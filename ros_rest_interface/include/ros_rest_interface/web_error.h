#pragma once

#include <stdexcept>

namespace web_http
{
    // Custom exception class for representing a request error
    class BadRequestError : public std::runtime_error
    {
    public:
        /*!
        * Constructor
        * @param message The error message to associate with the exception.
        */
        explicit BadRequestError(const std::string &message) : std::runtime_error(message) {}
        // Constructor that takes a message as input and initializes the base class std::runtime_error with the given message.
    };

    // Custom exception class for representing an internal error
    class InternalError : public std::runtime_error
    {
    public:
        /*!
        * Constructor
        * @param message The error message to associate with the exception.
        */
        explicit InternalError(const std::string &message) : std::runtime_error(message) {}
        // Constructor that takes a message as input and initializes the base class std::runtime_error with the given message.
    };
}
