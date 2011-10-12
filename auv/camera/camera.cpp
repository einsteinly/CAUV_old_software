#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include "camera.h"
#include "camera_observer.h"
#include "webcam.h"

#include <common/cauv_utils.h>
#include <utility/bash_cout.h>
#include <debug/cauv_debug.h>

using namespace cauv;

/* How camera sharing works:
 *
 * A client program that wants access to a camera connects to a magic port
 * (16708) on localhost, and sends 4 bytes which are the camera id (unsigned
 * integer) that it wishes to have access to.
 *
 * This server sends this response (native byte order):
 *  
 *  [uint32_t lock offset] 0..3
 *  [uint32_t image offset] 4..7
 *  [uint32_t image width] 8..11
 *  [uint32_t image height] 12..15
 *  [uint32_t image type] 16..19 (OpenCV type, eg CV_8UC3)
 *  [char] 5..
 *  [char]
 *   ...
 *  [char]
 *  [  0 ] N  shared memory segment name
 *
 * Once it has received this information, the client can get an image:
 *
 *   1) set *(shared mem + lock offset) = 1
 *   2) Send 4 bytes over the socket. Value doesn't matter
 *   // haven't decided yet:
 *   e: 3) poll *(shared mem + lock offset), until the value changes to 2
 *   or 3) wait for rx of 4 bytes on socket
 *
 *   Now an image is ready at *(shared mem + image offset). It will remain valid
 *   until the socket connection is closed (eg, in a crash), or *(shared mem +
 *   lock offset) is set to zero.
 *   
 *   The image must not be modified in place(!)
 *   
 *   4) set *(shared mem + lock offset) =  0
 *
 *  If another client requests an image from the camera within
 *  Max_Image_Latency, it will get the same image immediately (this case
 *  motivates the polling 3) above).
 *
 *  If the image is in use and older than Max_Image_Latency, clients that
 *  request an image will not be notified until a new one is available.
 *
 *  Note that this means that a process *could* block all access to cameras -
 *  if it deadlocked with a camera lock, for example. Relaxing this restriction
 *  would be possible by returning an offset for each image (either over the
 *  socket, or in the lock), presuming that the opencv camera input supports
 *  multiple active images at a time (which I'm pretty sure it does).
 *
 */


class CameraClientConnection{
        const static uint32_t Buf_Size = 64;
    public:
        CameraClientConnection(CameraServer& server, boost::asio::io_service &io_service)
            : m_camera_id(0), m_server(server), m_socket(io_service){
        }

         boost::asio::ip::tcp::socket& socket(){
            return m_socket;
        }

        void start(){
            _setupFirstRead();
        }

        void handleFirstRead(boost::system::error_code const& e, size_t s){
            if(!e){
                if(s == 4){
                    m_camera_id = *(static_cast<uint32_t*>(m_data));
                    m_server.didOpen(this, m_camera_id);
                }else{
                    error() << "CameraClientConnection::handleFirstRead: client didn't send 4 bytes";
                    delete this;
                }
            }else{
              debug() << "CameraClientConnection::handleFristRead failed: " << e << " " << e.message();
              // this is (certainly?) because the remote client crashed or
              // otherwise disconnected
              m_server.didClose(this);
            }
        }

        void handleRead(boost::system::error_code const& e){
            if (!e){
                // this is where complicated stuff happens
                //
                //  TTTTT OOO    DDD   OOO
                //    T  O   O   D  D O   O
                //    T   OOO    DDD   OOO
                //

                _setupRead();
            }else{
              debug() << "CameraClientConnection::handleRead failed: " << e << " " << e.message();
              // this is (certainly?) because the remote client crashed or
              // otherwise disconnected
              m_server.didClose(this);
            }
        }

    private:
        void _setupFirstRead(){
            m_socket.async_read_some(
                boost::asio::buffer(m_data, Buf_Size),
                boost::bind(
                    &CameraClientConnection::handleFirstRead, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred
                )
            );
        }

        void _setupRead(){
            m_socket.async_read_some(
                boost::asio::buffer(m_data, Buf_Size),
                boost::bind(
                    &CameraClientConnection::handleRead, this,
                    boost::asio::placeholders::error
                )
            );
        }

        uint32_t m_camera_id;
        
        CameraServer &m_server;
        boost::asio::ip::tcp::socket m_socket;
        char m_data[Buf_Size];
};

struct CameraProperties{
    uint32_t id;
    uint32_t resize_to_width;
    uint32_t resize_to_height;
    uint32_t convert_to_type;
};

class CameraManager{
    public:
        CameraManager(CameraProperties const& cam_properties)
            : m_segment(),
              m_image_ptr(NULL),
              m_image_handle(),
              m_locks(),
              m_capture(),
              m_cam_properties(cam_properties){
                // set up the shared memory for the camera image and locks:
                boost::interprocess::shared_memory_object::remove(shareName().c_str());
                uint32_t image_size = cam_properties.resize_to_width *
                                      cam_properties.resize_to_height *
                                      CV_ELEM_SIZE(cam_properties.type);
                uint32_t segment_size = image_size + Max_Clients * 4;
                segment_size += 1024;
                segment_size -= (segment_size % 1024);
                m_segment = boost::interprocess::managed_shared_memory(
                    boost::interprocess::create_only, shareName().c_str(), segment_size
                );
                m_image_ptr = m_segment.allocate(image_size);
                assert(m_image_ptr);
                m_image_handle = m_segment.get_handle_from_address(m_image_ptr);

                // set up the OpenCV capture
                // these don't always succeed, but if when we read a camera
                // image it isn't the right size, then we'll resize it then,
                // too:
                m_capture.set(CV_CAP_PROP_FRAME_WIDTH, cam_properties.resize_to_width);
                m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, cam_properties.resize_to_height);
                m_capture.open(cam_properties.id);
        }

        std::string shareName() const{
            return mkStr() << "Shared Camera " << 
        }

    private:
        boost::interprocess::managed_shared_memory m_segment;
        uint8_t* m_image_ptr;
        handle_t m_image_handle;
        
        std::map<CameraClientConnection*, std::pair<handle_t,uint32_t*> > m_locks;

        cv::VideoCapture m_capture;

        uint32_t m_cam_properties;
};

class CameraServer{
        typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
        typedef boost::asio::ip::tcp tcp;
        const static uint32_t Max_Clients = 64;
    public:
        CameraServer(boost::asio::io_service &service,
                     uint32_t port)
            : m_cams_and_clients(),
              m_service(service),
              m_acceptor(service, tcp::endpoint(tcp::v4(), port)){

                // prepare a connection to accept a client              
                _setupPendingConnection();
            }
                
        ~CameraServer(){
            boost::interprocess::shared_memory_object::remove(m_share_name.c_str());
        }

        void doAccept(CameraClientConnection *c, boost::system::error_code const& e){
            //std::cout << "Server::doAccept:" << c->socket().native() << " " << e << std::endl;
            if(!e){
                // start this connection, at some point later an asynchronous
                // call will (probably) be made back to didOpen()
                c->start();
                // prepare a new connection to accept another client
                _setupPendingConnection();
            }else{
                std::cerr << "Server::doAccept failed: " << e.message() << std::endl;
            }
        }

        void didOpen(CameraClientConnection *c, uint32_t camera_id){
            // !!! thread safety is not an issue because there is only one
            // worker thread for asio events
            std::map<uint32_t, CamManager_CamClientconns>::iterator i = m_cams_and_clients.find(camera_id);
            if(i == m_cams_and_clients.end()){
                CamManager_CamClientConns t;
                // !!! TODO: expose these via command line options, per camera id
                CameraProperties props = {
                    camera_id, 640, 480 CV_8UC3
                };
                t.manager = new CameraManager(props);
                m_cams_and_clients[camera_id] = t;
                i = m_cams_and_clients.find(camera_id);
            }
            i->second.connected.insert(c);
        }

        void didClose(CameraClientConnection *c, uint32_t camera_id){
            // !!! thread safety is not an issue because there is only one
            // worker thread for asio events
            std::map<uint32_t, CamManager_CamClientconns>::iterator i = m_cams_and_clients.find(camera_id);
            std::set<CameraClientConnection*>::iterator j;
            if(i == m_cams_and_clients.end() || !i->second.connected.erase(c))
                error() << "unmatched didClose!";
            else if(!i->second.connected.size()){
                debug() << "camera" << camera_id << "is now unused";
                delete i->second.manager;
                m_cams_and_clients.erase(i);
            }
        }

    private:
        void _setupPendingConnection(){
            uint32_t *lock_ptr = m_segment.allocate(4);
            handle_t lock_handle = handle_t(lock_ptr);
            CameraClientConnection *next_connection = new CameraClientConnection(*this, m_service, lock_ptr);
            m_locks[next_connection] = std::pair<handle_t, uint32_t*>(lock_handle, lock_ptr);
            m_acceptor.async_accept(
                next_connection->socket(),
                boost::bind(&Server::doAccept, this, next_connection, boost::asio::placeholders::error)
            );
        }
        
        // open cameras, and the clients connected to them
        struct CamManager_CamClientConns{
            CameraManager* manager;
            std::set<CameraClientConnection*> connected;
        };
        std::map<uint32_t, CamManager_CamClientConns> m_cams_and_clients;

        boost::asio::io_service &m_service;
        tcp::acceptor m_acceptor;
};





CameraException::CameraException(const std::string& _reason)
    : std::runtime_error(_reason)
{
}

ImageCaptureException::ImageCaptureException() : CameraException("Could not open camera device") {}


Camera::Camera(const CameraID::e id) : m_id(id)
{
}
Camera::~Camera()
{
}

CameraID::e Camera::id() const
{
    return m_id;
}




void Camera::broadcastImage(const cv::Mat& img)
{
    foreach(observer_ptr_t o, m_observers)
    {
        o->onReceiveImage(m_id, img);
    }
}



CaptureThread::CaptureThread(Webcam &camera, const int interFrameDelay)
	  : m_camera(camera), m_interFrameDelay(interFrameDelay), m_alive(true)
{
}

void CaptureThread::stop() {
    m_alive = false;  // Alert the image-capture loop that it needs to stop
}

void CaptureThread::setInterFrameDelay(const int delay) {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    m_interFrameDelay = delay;
}

int CaptureThread::getInterFrameDelay() const {
    boost::lock_guard<boost::mutex> guard(m_frameDelayMutex);
    return m_interFrameDelay;
}

void CaptureThread::operator()()
{
    while(m_alive)
    {   
        debug(3) << "Grabbing frame for broadcast...";
        m_camera.grabFrameAndBroadcast();
        
        m_frameDelayMutex.lock();
        int delay = m_interFrameDelay;
        m_frameDelayMutex.unlock();
        
        msleep(delay);
    }
}

Webcam::Webcam(const CameraID::e cameraID, const int deviceID)
    : Camera(cameraID), m_thread_callable(boost::ref(*this))
{
    //m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    //m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 280);
   
    //cv::namedWindow( "Webcam", CV_WINDOW_AUTOSIZE ); 
    
    // spot the deliberate mistake!
    m_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    m_capture = cv::VideoCapture(deviceID);
    if( !m_capture.isOpened() )
    {
        throw ImageCaptureException();
    }

    m_thread = boost::thread(boost::ref(m_thread_callable));
}
void Webcam::grabFrameAndBroadcast()
{
    cv::Mat mat;
    std::cout << "." << std::flush;
    m_capture >> mat;
//    cv::imshow("CAUV OpenCV test", mat);
//    cv::waitKey(10);
    broadcastImage(mat);
}

Webcam::~Webcam() {
    m_thread_callable.stop();
    m_thread.join();
}

