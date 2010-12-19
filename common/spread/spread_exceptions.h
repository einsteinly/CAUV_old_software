#ifndef CAUV_SPREAD_EXCEPTIONS_H_INCLUDED
#define CAUV_SPREAD_EXCEPTIONS_H_INCLUDED

#include <stdexcept>
#include <string>

inline static std::string getErrorString(int err)
{
    switch( err )
	{
		case ILLEGAL_SPREAD:
			return "Spread error: Illegal spread was provided";
		case COULD_NOT_CONNECT:
			return "Spread error: Could not connect. Is Spread running?";
		case REJECT_QUOTA:
			return "Spread error: Connection rejected, to many users";
		case REJECT_NO_NAME:
			return "Spread error: Connection rejected, no name was supplied";
		case REJECT_ILLEGAL_NAME:
			return "Spread error: Connection rejected, illegal name";
		case REJECT_NOT_UNIQUE:
			return "Spread error: Connection rejected, name not unique";
		case REJECT_VERSION:
			return "Spread error: Connection rejected, library does not fit daemon";
		case CONNECTION_CLOSED:
			return "Spread error: Connection closed by spread";
		case REJECT_AUTH:
			return "Spread error: Connection rejected, authentication failed";
		case ILLEGAL_SESSION:
			return "Spread error: Illegal session was supplied";
		case ILLEGAL_SERVICE:
			return "Spread error: Illegal service request";
		case ILLEGAL_MESSAGE:
			return "Spread error: Illegal message";
		case ILLEGAL_GROUP:
			return "Spread error: Illegal group";
		case BUFFER_TOO_SHORT:
			return "Spread error: The supplied buffer was too short";
		case GROUPS_TOO_SHORT:
			return "Spread error: The supplied groups list was too short";
		case MESSAGE_TOO_LONG:
			return "Spread error: The message body + group names was too large to fit in a message";
		case NET_ERROR_ON_SESSION:
			return "Spread error: The network socket experienced an error. This Spread mailbox will no longer work until the connection is disconnected and then reconnected";
		default:
			return "Spread error: Unrecognized error";
	}
}
inline static bool isCritical(int err)
{
    switch (err)
    {
        case ILLEGAL_SPREAD:
        case CONNECTION_CLOSED:
        case REJECT_VERSION:
        case REJECT_NO_NAME:
        case REJECT_ILLEGAL_NAME:
		case NET_ERROR_ON_SESSION:
            return true;
        default:
            return false;
    }
}

inline static bool needsReconnect(int err)
{
    switch(err)
    {
        case CONNECTION_CLOSED:
        case NET_ERROR_ON_SESSION:
            return true;
        default:
            return false;
        
    }
}

class ConnectionError : public std::runtime_error {
    public:
        explicit ConnectionError(const std::string& msg,
                                 bool critical = false,
                                 bool needs_reconnect = true)
            : std::runtime_error(msg), m_err(-1), m_critical(critical),
              m_needs_reconnect(needs_reconnect){
        }

        explicit ConnectionError(const int error)
            : std::runtime_error(getErrorString(error)), m_err(error),
              m_critical(isCritical(error)), m_needs_reconnect(false){
        }
        
        bool critical() const {
            return m_critical;
        }
        
        bool needsReconnect() const{
            return m_needs_reconnect;
        }

        int err() const{
            return m_err;
        }

    private:
        int m_err;
        bool m_critical;
        bool m_needs_reconnect;
};

#endif // CAUV_SPREAD_EXCEPTIONS_H_INCLUDED
