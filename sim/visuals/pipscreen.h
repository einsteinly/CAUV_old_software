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
