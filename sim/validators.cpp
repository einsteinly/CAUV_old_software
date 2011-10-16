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

#include "validators.h"

#include <osg/Vec3f>
#include <osg/Vec2f>

namespace osg {

    void validate(boost::any& v, const std::vector<std::string>& values, osg::Vec3f*, int)  {
        osg::Vec3f vec(
                boost::lexical_cast<float>(values[0]),
                boost::lexical_cast<float>(values[1]),
                boost::lexical_cast<float>(values[2])
                );
        v = vec;
    }

    void validate(boost::any& v, const std::vector<std::string>& values, osg::Vec2f*, int)  {
        osg::Vec2f vec(
                boost::lexical_cast<float>(values[0]),
                boost::lexical_cast<float>(values[1])
                );
        v = vec;
    }
} // namespace osg
