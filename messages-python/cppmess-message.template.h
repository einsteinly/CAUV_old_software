/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_MESSAGE_MESSAGE_H__
\#define __CAUV_MESSAGE_MESSAGE_H__

\#include <string>
\#include <sstream>
\#include <boost/cstdint.hpp>
\#include <utility/serialisation-types.h>

namespace cauv{

// ================
// Type definitions
// ================

// Base message class
class Message
{
    public:
        virtual ~Message();

        std::string const& group() const;
        uint32_t id() const;

        virtual const_svec_ptr toBytes() const = 0;

    protected:
        uint32_t m_id;
        std::string m_group;

        Message(uint32_t id, std::string const& group);

        virtual std::string _str() const; 

    template <typename char_T, typename traits>
    friend std::basic_ostream<char_T, traits>& operator<<(
        std::basic_ostream<char_T, traits>& os, Message const& m);

};

template <typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Message const& m)
{
    return os << m._str(); 
}

} // namespace cauv

\#endif // __CAUV_MESSAGE_MESSAGE_H__

