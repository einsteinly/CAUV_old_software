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

#ifndef __CAUV_QT_STREAMOPS_H__
#define __CAUV_QT_STREAMOPS_H__

#include <QPointF>
#include <QSizeF>
#include <QRectF>

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QPointF const& p){
    return os << "(" << p.x() << "," << p.y() << ")";
}

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QSizeF const& s){
    return os << "(" << s.width() << "x" << s.height() << ")";
}

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QRectF const& s){
    return os <<"("<< s.x() <<","<< s.y() <<":"<< s.width() <<"x"<< s.height() <<")";
}


#endif //ndef __CAUV_QT_STREAMOPS_H__
