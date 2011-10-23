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

#ifndef __CAUV_SERIALISATION_TYPES_H__
#define __CAUV_SERIALISATION_TYPES_H__

#include <vector>

#include <boost/shared_ptr.hpp>

namespace cauv{

typedef unsigned char byte;
typedef std::vector<byte> svec_t;
typedef boost::shared_ptr<svec_t> svec_ptr;
typedef boost::shared_ptr<const svec_t> const_svec_ptr;

} // namespace cauv

#endif // ndef __CAUV_SERIALISATION_TYPES_H__

