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

#include "camera.h"

using namespace cauv;
using namespace cauv::sim;

#include <boost/make_shared.hpp>

#include <osgDB/WriteFile>

#include <common/cauv_utils.h>

#include <generated/types/ImageMessage.h>
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

    cv::Mat data = cv::Mat(m_height, m_width, CV_8UC3, (uchar*)this->data(), 0);
    cv::Mat mat(m_height, m_width, CV_8UC3);
    cv::flip(data, mat, 1);
    boost::shared_ptr<ImageMessage> msg = boost::make_shared<ImageMessage>(CameraID::Forward, cauv::Image(mat), cauv::now());
    m_simulator->send(msg);

    //std::stringstream str2;
    //str2 << "frames/" << simTime << ".png";
    //osgDB::writeImageFile(*this,str2.str());

    // TODO: send image signal

    if(m_rateLimiter){
        m_rateLimiter->click();
    }
}
