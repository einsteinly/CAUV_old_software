#include "buoy.h"

cauv::BuoyNode::BuoyNode(float radius) : 
        osg::Geode(),
        sphere(new osg::Sphere(osg::Vec3f(), radius)),
        drawable(new osg::ShapeDrawable(sphere.get())) {
    drawable->setColor(osg::Vec4f(1,1,0,1));
    addDrawable(drawable);
};
