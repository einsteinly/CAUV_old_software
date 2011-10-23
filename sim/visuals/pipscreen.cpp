/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "pipscreen.h"

#include <osg/StateSet>
#include <osg/Texture2D>
#include <osg/Geometry>

using namespace cauv;
using namespace cauv::sim;

PipScreen::PipScreen(osg::ref_ptr<osg::Texture2D> texture, int width, int height)
{
    m_screenQuad = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                   osg::Vec3(width, 0.0, 0.0),
                                                   osg::Vec3(0.0, height, 0.0));
    this->addDrawable(m_screenQuad.get());

    setTexture(texture.get());
}

void PipScreen::setTexture(osg::ref_ptr<osg::Texture2D> texture){
    osg::StateSet *quadState = this->getOrCreateStateSet();
    quadState->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
}
