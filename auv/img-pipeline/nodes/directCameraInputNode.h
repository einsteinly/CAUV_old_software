/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __DIRECT_CAMERA_INPUT_NODE_H__
#define __DIRECT_CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "asynchronousNode.h"

#include <generated/types/SensorUIDBase.h>

#define MAX_DEVICES 5

namespace cauv{
namespace imgproc{

class DirectCameraInputNode: public AsynchronousNode{
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        DirectCameraInputNode(ConstructArgs const& args)
            : AsynchronousNode(args),
              m_current_device(-1),
              m_seq(0){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter:
            registerParamID<int>("device id", 0);
            
            openCapture();
        }

        virtual ~DirectCameraInputNode(){
            stop();
            m_capture = cv::VideoCapture();
            if(m_current_device != -1)
                m_capture_lock[m_current_device].unlock();
        }

        virtual void paramChanged(input_id const& p){
            if(p == "device id")
            {
                openCapture();
            }
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "DirectCameraInputNode::doWork";
            
            if(!m_capture.isOpened()){
                error() << "camera is not opened";
            }else{
                msleep(10);
                
                cv::Mat img;
                m_capture >> img;
                // use internalValue to avoid automatic UID setting on outputs                
                r.internalValue("image_out") = boost::make_shared<Image>(img, now(), mkUID(SensorUIDBase::Camera + m_current_device, ++m_seq));
            }
        }

    private:

        void openCapture(){
            #ifdef __APPLE__
            warning() << "Direct camera access is not possible on OS X: use shared (normal) cameras";
            #else
            int dev_id = param<int>("device id");
            if(dev_id >= MAX_DEVICES || dev_id < 0){
                error() << "invalid camera:" << dev_id;
            }else{
                bool l = m_capture_lock[dev_id].try_lock();
                if(!l){
                    error() << "camera already open" << dev_id;
                }else{
                    if(m_current_device != -1)
                        m_capture_lock[m_current_device].unlock();
                    m_current_device = dev_id;
                    try{
                        m_capture = cv::VideoCapture(dev_id);
                    }catch(cv::Exception& e){
                        error() << "capture exception:" << e.what();
                    }
                   
                    if(!m_capture.isOpened()){
                        error() << "could not open camera" << dev_id;
                        return;
                    } 
                    //m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
                    //m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
                    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
                    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
                    setAllowQueue();
                }
            }
            #endif
        }
        
        cv::VideoCapture m_capture;
        int m_current_device;
        uint64_t m_seq;
        static boost::try_mutex m_capture_lock[MAX_DEVICES];
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DIRECT_CAMERA_INPUT_NODE_H__

