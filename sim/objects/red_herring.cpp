#include "red_herring.h"

cauv::RedHerringNode::RedHerringNode(void) :
        osg::Geode(),
        box(new osg::Box(osg::Vec3f(), 0.2, 1, 0.2)),
        front_marker(new osg::Box(osg::Vec3f(0,0.5,0),0.1, 0.1, 0.1)),
        drawable(new osg::ShapeDrawable(box.get())),
    front_drawable(new osg::ShapeDrawable(front_marker.get())) {
    drawable->setColor(osg::Vec4f(1,0,0,1));
    addDrawable(drawable);
    addDrawable(front_drawable);
}
