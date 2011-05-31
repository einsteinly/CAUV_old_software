#include "camera.h"

using namespace cauv;
using namespace cauv::sim;

#include <osgDB/WriteFile>

#include <common/cauv_utils.h>

#include <sstream>

Camera::Camera(int width, int height)
{
    setSize(width, height);
}

void Camera::setSize(int width, int height){
    m_width = width;
    m_height = height;

    this->allocateImage(width, height, 24, GL_RGB, GL_UNSIGNED_BYTE);
}

void Camera::setRateLimiter(boost::shared_ptr<RateLimiter> rateLimiter){
    m_rateLimiter = rateLimiter;
}

void Camera::tick(double simTime){
    // rate limit if requested
    if(m_rateLimiter && m_rateLimiter->isSaturated())
        return;

    std::stringstream str2;
    str2 << "frames/" << simTime << ".png";
    osgDB::writeImageFile(*this,str2.str());

    // TODO: send image signal

    if(m_rateLimiter){
        m_rateLimiter->click();
    }
}
