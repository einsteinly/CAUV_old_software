/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

/***  This is a generated file, do not edit ***/
\#include "message.h"

cauv::Message::Message(uint32_t id, std::string const& group) : m_id(id), m_group(group) { }

cauv::Message::~Message() { }

std::string const& cauv::Message::group() const{
    return m_group;
}

uint32_t cauv::Message::id() const{
    return m_id;
}

std::string cauv::Message::_str() const {
    std::stringstream ss;
    ss << "Unknown message {";
    ss << " id = " << std::dec << m_id << ",";
    ss << " group = " << m_group;
    ss << " }";
    return ss.str();
}
