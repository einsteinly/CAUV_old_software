#ifndef CAUV_WATER_OBJ_H
#define CAUV_WATER_OBJ_H

#include <osg/Geode>
#include <osg/ShapeDrawable>

namespace cauv {

class WaterNode : public osg::Geode {
    public:
    WaterNode(void);
    private:
    osg::ref_ptr<osg::Box> plane;
    osg::ref_ptr<osg::ShapeDrawable> drawable;
};

}

#endif
