#ifndef CAMERA_H
#define CAMERA_H

#include <osgViewer/View>

#include <boost/shared_ptr.hpp>

#include <osg/Image>

#include <sim/simnode.h>

namespace osg {
    class Image;
    class GraphicsContext;
}

namespace cauv {

    class RateLimiter;

    namespace sim {

        class Camera : public SimNode, public osg::Image
        {
        public:
            Camera(int width, int height);

            void tick(double simTime);
            void setSize(int width, int height);
            void setRateLimiter(boost::shared_ptr<RateLimiter> rateLimiter);

        protected:
            int m_width;
            int m_height;
            boost::shared_ptr<RateLimiter> m_rateLimiter;
        };

    } // namespace sim

} // namespace cauv
#endif // CAMERA_H
