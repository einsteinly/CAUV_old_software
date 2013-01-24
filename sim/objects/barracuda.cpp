/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "barracuda.h"

cauv::BarracudaNode::BarracudaNode(void) :
        osg::Geode(),
        box(new osg::Box(osg::Vec3f(), 0.15, 1.5, 0.15)),
        front_marker(new osg::Box(osg::Vec3f(0,0.5,0),0.1, 0.1, 0.1)),
        drawable(new osg::ShapeDrawable(box.get())),
    front_drawable(new osg::ShapeDrawable(front_marker.get())) {
    drawable->setColor(osg::Vec4f(0,0,0,1));
    addDrawable(drawable);
    addDrawable(front_drawable);
}
