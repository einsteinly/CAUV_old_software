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
