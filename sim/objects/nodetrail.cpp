/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "nodetrail.h"

#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <algorithm>

osg::StateSet* initVTState()
{
     osg::StateSet* svss = new osg::StateSet();
     osg::PolygonMode* polymode = new osg::PolygonMode;
     polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
     svss->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
     osg::LineWidth *lineWidth = new osg::LineWidth; 
     lineWidth->setWidth(1.0f); 
     svss->setAttribute(lineWidth); 
     svss->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
     return svss;
}

NodeTrail::NodeTrail(osg::Node *node_, osg::Vec3 offset_,
                     osg::Vec3 color_, unsigned int length_) :
    node(node_),
    geometry(new osg::Geometry()),
    primitive(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 0)),
    verts(new osg::Vec3Array()),
    color(new osg::Vec3Array()),
    offset(offset_),
    length(length_) {
    geometry->addPrimitiveSet(primitive);
    geometry->setVertexArray(verts);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    color->push_back(color_);
    geometry->setColorArray(color);
    addDrawable(geometry);
    geometry->setUseDisplayList(false);
    setStateSet(initVTState());
}

void NodeTrail::tick(void) {
    osg::Matrixd local_to_world;
    local_to_world = osg::computeLocalToWorld(node->getParentalNodePaths()[0]);
    osg::Vec3 new_point = offset * local_to_world;
    if (verts->size()) {
        osg::Vec3 old_point = (*verts)[verts->size() - 1];
        if ((new_point - old_point).length() < 0.05) {
            return;
        }
    }
    addPoint(new_point);
}

void NodeTrail::addPoint(osg::Vec3 point) {
    verts->push_back(point);
    if (verts->size() >= 2 * length) {
        std::copy(verts->end() - length, verts->end(), verts->begin());
        verts->erase(verts->end() - length, verts->end());
        primitive->setFirst(0);
    }
    primitive->setCount(verts->size());
    if (verts->size() >= length) {
        primitive->setFirst(verts->size() - length);
        primitive->setCount(length);
    }
}
