#ifndef CAUV_NODETRAIL_H
#define CAUV_NODETRAIL_H

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>

class NodeTrail : public osg::Geode {
    public:
    NodeTrail(osg::Node *node, osg::Vec3 offset,
              osg::Vec3 color, unsigned int length);

    void tick(void);

    private:
    void addPoint(osg::Vec3 point);
    osg::ref_ptr<osg::Node> node;
    osg::ref_ptr<osg::Geometry> geometry;
    osg::ref_ptr<osg::DrawArrays> primitive;
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec3Array> color;
    osg::Vec3 offset;
    unsigned int length;
};

#endif
