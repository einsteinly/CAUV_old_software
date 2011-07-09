/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${g.name.upper()}_GROUP_H__
\#define __CAUV_${g.name.upper()}_GROUP_H__

#for $m in $g.messages
\#include "${m.name}Message.h"
#end for

\#endif // __CAUV_${g.name.upper()}_GROUP_H__
