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

#ifndef __CAUV_WEBCAM_H__
#define __CAUV_WEBCAM_H__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cauv{

class Webcam : public Camera
{
    public:
        Webcam(const CameraID::e cameraID, const int deviceID);
        virtual ~Webcam();
    
    protected:
        cv::VideoCapture m_capture;
        CaptureThread m_thread_callable;
        boost::thread m_thread;
        void grabFrameAndBroadcast();
        
    friend class CaptureThread;
};

} // namespace cauv

#endif // ndef __CAUV_WEBCAM_H__

