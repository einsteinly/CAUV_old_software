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

#ifndef PIPSCREEN_H
#define PIPSCREEN_H

#include <osg/ref_ptr>
#include <osg/Geode>

namespace osg {
    class Geometry;
    class Texture2D;
}

namespace cauv {

    namespace sim {

        class PipScreen : public osg::Geode
        {

        public:
            PipScreen(osg::ref_ptr<osg::Texture2D> texture, int width, int height);

            void setTexture(osg::ref_ptr<osg::Texture2D> texture);

        protected:
            osg::ref_ptr<osg::Geometry> m_screenQuad;

        };

    } //namespace sim

}// namespace cauv

#endif // PIPSCREEN_H
