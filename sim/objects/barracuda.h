#ifndef CAUV_BARRACUDA_OBJ_H
#define CAUV_BARRACUDA_OBJ_H

#include <osg/Geode>
#include <osg/ShapeDrawable>

namespace cauv {

class BarracudaNode : public osg::Geode {
    public:
    BarracudaNode(void);
    private:
    osg::ref_ptr<osg::Box> box;
    osg::ref_ptr<osg::Box> front_marker;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
    osg::ref_ptr<osg::ShapeDrawable> front_drawable;
};

}

#endif
