#ifndef CAMERA_H
#define CAMERA_H

#include <osgViewer/View>

namespace osg {
    class Texture2D;
    class Image;
}

namespace cauv {

    namespace sim {

        class Camera : public osgViewer::View
        {
        public:
            Camera(int width = 200, int height = 200);

            void setUpAsHUD();


            void setSize(int width, int height);

            osg::ref_ptr<osg::Texture2D> getTexture();
            osg::ref_ptr<osg::Image> getImage();

        protected:
            osg::ref_ptr<osg::Texture2D> m_renderTexture;
        };

    } // namespace sim

} // namespace cauv
#endif // CAMERA_H
