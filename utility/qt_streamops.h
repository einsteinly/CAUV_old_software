#ifndef __CAUV_QT_STREAMOPS_H__
#define __CAUV_QT_STREAMOPS_H__

#include <QPointF>

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QPointF const& p){
    return os << "(" << p.x() << "," << p.y() << ")";
}


#endif //ndef __CAUV_QT_STREAMOPS_H__
