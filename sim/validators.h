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

#ifndef VALIDATORS_H
#define VALIDATORS_H

#include <boost/program_options.hpp>

namespace osg {

    class Vec3f;
    void validate(boost::any& v, const std::vector<std::string>& values, osg::Vec3f*, int);

    class Vec2f;
    void validate(boost::any& v, const std::vector<std::string>& values, osg::Vec2f*, int);
}


#endif // VALIDATORS_H
