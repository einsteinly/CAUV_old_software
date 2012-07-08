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

#ifndef __CAUV_UTILITY_ARRAYS_H__
#define __CAUV_UTILITY_ARRAYS_H__

#define MAX_ARRAY_SIZE 10

#include <boost/array.hpp>
#include <boost/preprocessor/repetition.hpp>

namespace cauv {

#define MAKE_ARRAY(z, N, unused) \
template<typename T> \
boost::array<T,N> make_array(BOOST_PP_ENUM_PARAMS(N,T val)) { \
    boost::array<T,N> ret = {{ BOOST_PP_ENUM_PARAMS(N,val) }}; \
    return ret; \
}

BOOST_PP_REPEAT(MAX_ARRAY_SIZE, MAKE_ARRAY, ~)

#undef MAKE_ARRAY

}

#endif
