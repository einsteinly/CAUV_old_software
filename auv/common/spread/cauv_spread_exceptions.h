#ifndef CAUV_SPREAD_EXCEPTIONS_H_INCLUDED
#define CAUV_SPREAD_EXCEPTIONS_H_INCLUDED

#include <stdexcept>
#include <string>

class ConnectionError : public std::runtime_error {
public:
    explicit ConnectionError(const std::string &what, bool critical = false)
        : std::runtime_error(what), m_critical(critical){}
    
    bool critical() const {
        return m_critical;
    }
    
private:
    bool m_critical;
};

class InvalidSessionError : public std::logic_error {
public:
    explicit InvalidSessionError(const std::string &what = "Mailbox not connected")
        : std::logic_error(what){}
};

class IllegalGroupError : public std::logic_error {
public:
    explicit IllegalGroupError(const std::string &what = "Group name too long or too short")
        : std::logic_error(what){}
};

class IllegalMessageError : public std::logic_error {
public:
    explicit IllegalMessageError(const std::string &what = "Illegal messsage structure")
        : std::logic_error(what){}
};

#endif // CAUV_SPREAD_EXCEPTIONS_H_INCLUDED
