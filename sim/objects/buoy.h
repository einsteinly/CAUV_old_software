#ifndef CAUV_BUOY_OBJ
#define CAUV_BUOY_OBJ

#include <osg/Geode>
#include <osg/ShapeDrawable>

namespace cauv {

class BuoyNode : public osg::Geode {
    public:
    BuoyNode(float radius); 

    private:
    osg::ref_ptr<osg::Sphere> sphere;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
};

}

#endif
