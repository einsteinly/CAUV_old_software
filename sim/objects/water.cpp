#include "water.h"
#include <osg/BlendFunc>

cauv::WaterNode::WaterNode(void) :
        osg::Geode(),
        plane(new osg::Box(osg::Vec3f(0,0,-10),80,80,20)),
        drawable(new osg::ShapeDrawable(plane.get())) {
    addDrawable(drawable);
    drawable->setColor(osg::Vec4f(1,0.1,1,0.3));
    osg::StateSet* set = getOrCreateStateSet();
    set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    osg::ref_ptr<osg::BlendFunc> blend = new osg::BlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA);
    set->setAttributeAndModes(blend, osg::StateAttribute::ON);
}
