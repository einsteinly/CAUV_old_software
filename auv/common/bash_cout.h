#ifndef __BASH_COUT_H__
#define __BASH_COUT_H__

#include <iomanip>
#include <string>
#include <boost/assign.hpp>


using boost::assign::map_list_of;


static std::map<std::string, int> col_to_colstr = map_list_of
    ("black", 30)
    ("red",   31)
    ("green", 32)
    ("brown", 33)
    ("blue",  34)
    ("purple",35)
    ("cyan",  36)
    ("white", 37)
;



struct _SetColour { int colint; };
inline _SetColour setcolour(std::string col)
{
    _SetColour s;
    s.colint = col_to_colstr[col];
    if (s.colint == 0)
        s.colint = -1;
    return s;
}
inline _SetColour setbg(std::string col)
{
    _SetColour s = setcolour(col);
    if (s.colint != -1)
        s.colint += 10;
    return s;
}
template<typename _CharT, typename _Traits>
inline std::basic_ostream<_CharT, _Traits>& operator<< (std::basic_ostream<_CharT, _Traits>& os, _SetColour f)
{ 
    if (f.colint >= 0)
        os << "\E[;" << f.colint << "m"; 
    return os; 
}

template<int i> // This template is a nasty hack so that I don't need a cpp file
struct _SetBashAttr {
    std::string s;
    _SetBashAttr(std::string s) : s(s) {}
};
_SetBashAttr<0> resetcolour("\E[m");
_SetBashAttr<0> setbold("\E[1m");


template<typename _CharT, typename _Traits>
inline std::basic_ostream<_CharT, _Traits>& operator<< (std::basic_ostream<_CharT, _Traits>& os, _SetBashAttr<0> f)
{ 
  os << f.s; 
  return os; 
}

#endif//__BASH_COUT_H__
