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
