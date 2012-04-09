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
\#ifndef __CAUV_${g.name.upper()}_GROUP_H__
\#define __CAUV_${g.name.upper()}_GROUP_H__

#for $m in $g.messages
\#include "${m.name}Message.h"
#end for

\#endif // __CAUV_${g.name.upper()}_GROUP_H__
