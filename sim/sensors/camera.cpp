#include "camera.h"

using namespace cauv;
using namespace cauv::sim;

#include <osg/Texture2D>
#include <osg/Image>


Camera::Camera(int width, int height) : m_renderTexture(new osg::Texture2D)
{
    setSize(width, height);
    m_renderTexture->setInternalFormat(GL_RGBA);
    m_renderTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    m_renderTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);


    setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // We need to render to the texture BEFORE we render to the screen
    setRenderOrder(osg::Camera::PRE_RENDER);

    // The camera will render into the texture that we created earlier
    attach(osg::Camera::COLOR_BUFFER, m_renderTexture);

    setAllowEventFocus(false);
}

void Camera::setSize(int width, int height){
    m_renderTexture->setTextureSize(width, height);
    setViewport(0, 0, width, height);
}

osg::ref_ptr<osg::Texture2D> Camera::getTexture(){
    return m_renderTexture;
}

osg::ref_ptr<osg::Image> Camera::getImage(){
    return m_renderTexture->getImage();
}
