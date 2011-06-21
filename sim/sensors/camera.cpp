#include "camera.h"

using namespace cauv;
using namespace cauv::sim;

#include <boost/make_shared.hpp>

#include <osgDB/WriteFile>

#include <common/cauv_utils.h>

#include <generated/messages.h>
#include <opencv/cv.h>

#include <sstream>

Camera::Camera(Simulator * s, int width, int height) : SimNode(s)
{
    setSize(width, height);
}

void Camera::setSize(int width, int height){
    m_width = width;
    m_height = height;

    this->allocateImage(width, height, 1, GL_BGR, GL_UNSIGNED_BYTE);

    this->setOrigin(osg::Image::BOTTOM_LEFT);
}

void Camera::setRateLimiter(boost::shared_ptr<RateLimiter> rateLimiter){
    m_rateLimiter = rateLimiter;
}

void Camera::tick(double simTime){
    // rate limit if requested
    if(m_rateLimiter && m_rateLimiter->isSaturated())
        return;

    cv::Mat mat = cv::Mat(m_height, m_width, CV_32FC1, (uchar*)this->data(), 0);
    cvFlip(&mat, NULL, 1);
    boost::shared_ptr<ImageMessage> msg = boost::make_shared<ImageMessage>(CameraID::Forward, mat, cauv::now());
    m_simulator->send(msg);

    //std::stringstream str2;
    //str2 << "frames/" << simTime << ".png";
    //osgDB::writeImageFile(*this,str2.str());

    // TODO: send image signal

    if(m_rateLimiter){
        m_rateLimiter->click();
    }
}
