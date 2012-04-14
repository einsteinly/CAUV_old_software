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

#ifndef __CAUV_LOCK_MACRO_H__
#define __CAUV_LOCK_MACRO_H__
#include <boost/typeof/typeof.hpp>

#define CAUV_LOCK(MUTEX) if (bool _lock_guard_bool = false) {} \
    else for (boost::lock_guard<BOOST_TYPEOF(MUTEX)> _lock_guard(MUTEX); !_lock_guard_bool; _lock_guard_bool = true)

#endif
