/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAMERA_INPUT_NODE_H__
#define __CAMERA_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera/client.h>
#include <camera/server_shared.h>

#include <generated/types/SensorUIDBase.h>

#include "asynchronousNode.h"


#define MAX_DEVICES 5

namespace cauv{
namespace imgproc{

class CameraInputNode: public AsynchronousNode{
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        CameraInputNode(ConstructArgs const& args)
            : AsynchronousNode(args),
              m_server_connection(),
              m_seq(0){
            setAllowQueue();
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image_out", image_ptr_t());
            
            // parameters:
            registerParamID<int>("device id", 0);
            registerParamID<int>("width", 640);
            registerParamID<int>("height", 480);
        }

    protected:
        struct SharedImageDeleter{
            // if the connection were to close whilst an image was active, then
            // the memory is unmapped, and we'd get EXC_BAD_ACCESS trying to
            // set the lock, or access the image - so pass a shared pointer to
            // the connection around to keep it alive
            SharedImageDeleter(SharedImage* s, boost::shared_ptr<CameraServerConnection> c)
                : m_lock_ptr(&(s->lock)), m_connection(c){
            }

            void operator()(Image* img){
                *m_lock_ptr = 0;
                delete img;
                m_connection.reset();
            }

            volatile int32_t* m_lock_ptr;
            boost::shared_ptr<CameraServerConnection> m_connection;
        };

        void doWork(in_image_map_t&, out_map_t& r){

            if(!m_server_connection)
                m_server_connection = boost::make_shared<CameraServerConnection>();

            int camera_id = param<int>("device id");
            int w = param<int>("width");
            int h = param<int>("height");
            
            debug(4) << "CameraInputNode::doWork";
            
            SharedImage *s = m_server_connection->getUnGuardedImage(camera_id, w, h);
            
            // use internalValue to avoid automatic UID setting on outputs
            r.internalValue("image_out") = boost::shared_ptr<Image>(
                new Image(
                    cv::Mat(s->height, s->width, s->type, &(s->bytes[0]), s->pitch),
                    now(),
                    mkUID(SensorUIDBase::Camera + camera_id, ++m_seq)
                ),
                SharedImageDeleter(s, m_server_connection)
            );

        }

    protected:

        boost::shared_ptr<CameraServerConnection> m_server_connection;
        uint64_t m_seq;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CAMERA_INPUT_NODE_H__

