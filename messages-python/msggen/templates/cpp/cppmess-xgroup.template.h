/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${g.name.upper()}_GROUP_H__
\#define __CAUV_${g.name.upper()}_GROUP_H__

#for $m in $g.messages
\#include "${m.name}Message.h"
#end for

\#endif // __CAUV_${g.name.upper()}_GROUP_H__
