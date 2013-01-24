/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_CAMERA_SERVER_H__
#define __CAUV_CAMERA_SERVER_H__

#include <list>
#include <map>
#include <set>

#include <boost/asio.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/shared_ptr.hpp> 

#include <opencv2/highgui/highgui.hpp>

#include "server_shared.h"

#ifdef CAUV_USE_DC1394
#include <dc1394/dc1394.h>
#endif // def CAUV_USE_DC1394 

/* How camera sharing works:
 *
 * A client program that wants access to a camera connects to a magic port
 * (CAMERA_SERVER_PORT) on localhost.
 *
 * The name of the shared memory segment used is uk.co.cauv.shared.cameras
 *
 * Each time an image is desired, the following process occurs:
 *  1) Client sends an ImageRequest structure over the socket
 *  2) This server sends an InfoResponse structure in response
 *
 * Once it has received this information, the client can get an image:
 *  3) resolve image_offset into a pointer to a SharedImage structure
 *  4) do stuff with the image
 *  5) set shared_image.lock = 0 when finished with the image. The server may
 *     then free the memory.
 *
 * In the current implementation a unique image is returned for each request.
 * It may be advantageous to share images (if two requests are almost
 * coincident), if performance dictates this.
 *
 */

namespace cauv{

class CameraServer;
class CameraManager;

typedef boost::asio::ip::tcp tcp;
namespace bs = boost::system;

class CameraClientConnection{
    public:
        CameraClientConnection(CameraServer& server,
                               CameraManager& manager,
                               boost::asio::io_service &io_service);
        ~CameraClientConnection();

        tcp::socket& socket();

        void start();

        void handleRead(bs::error_code const& e, size_t s);
        void handleWrite(bs::error_code const& e);

    private:
        void _setupRead();

        CameraManager &m_camera_manager;
        std::list<uint64_t> m_current_images;
        
        CameraServer &m_server;
        tcp::socket m_socket;
        InfoResponse m_temp_response_data;
        ImageRequest m_temp_image_request;
};

class CameraManager{
        typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
        // used to determine the size of shared memory segment to create
        const static uint32_t Max_Concurrent_Images = 10;
    public:
        CameraManager();
        ~CameraManager();
        
        /* getImage:
         *      resp: (out) set to contain information about where in shared
         *                  memory to find the image grabbed from the camera.
         *      active_images: (in/out) 
         *                  each active image is checked to see if it can be
         *                  freed, and the list modified and returned if any
         *                  are freed: this allows the new image to be
         *                  allocated in the same place as the last one in the
         *                  common case that a single image is active at a
         *                  time.
         *                  The new image is included in the list. (there is no
         *                  need for the caller to add it)
         *      req: (in)   information about what image is desired
         *      c: (in)     An internal record is kept of which connections
         *                  have used which cameras, when all the connections
         *                  that have used a camera close, the camera is
         *                  closed.
         */
        void getImage(InfoResponse& resp,
                      std::list<uint64_t>& active_images,
                      ImageRequest const& req,
                      CameraClientConnection *c);
        
        /* Record that c has closed, and that it should not be regarded as
         * using any cameras any more.
         */
        void didClose(CameraClientConnection *c);
        
        /* release:
         *   Release all images in the list, the image locks are NOT checked.
         */
        void release(std::list<uint64_t> const& images);

    private:
        class Capture{
            public:
                virtual void captureToMem(
                    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
                ) = 0;
                virtual ~Capture(){}
                virtual bool ok() const = 0;
        };
        class CVCapture: public Capture, public cv::VideoCapture{
            public:
                CVCapture(int32_t cam_id);
                virtual void captureToMem(
                    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
                );
                virtual bool ok() const;
        };
        #ifdef CAUV_USE_DC1394
        class DC1394Capture: public Capture{
            public:
                DC1394Capture(ImageRequest const& req);
                virtual ~DC1394Capture();
                virtual void captureToMem(
                    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
                );
                virtual bool ok() const;
            private:
                dc1394_t* m_dc1394;
                dc1394camera_t *m_camera;
                dc1394video_mode_t m_video_mode;
        };
        #endif // def CAUV_USE_DC1394

        static boost::shared_ptr<Capture> openCam(ImageRequest const& req);
        

        boost::interprocess::managed_shared_memory m_segment;
        std::map<uint32_t, boost::shared_ptr<Capture> > m_open_cameras;
        std::map<uint32_t, std::set<CameraClientConnection*> > m_camera_users;
};

class CameraServer{
    public:
        CameraServer(boost::asio::io_service &service, uint32_t port);

        void doAccept(CameraClientConnection *c, bs::error_code const& e);

    private:
        void _setupPendingConnection();
        
        CameraManager m_manager;
        boost::asio::io_service &m_service;
        tcp::acceptor m_acceptor;
};

} // namespace cauv

#endif // ndef __CAUV_CAMERA_SERVER_H__

