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
