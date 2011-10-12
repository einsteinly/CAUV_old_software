#include <list>
#include <map>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <common/cauv_utils.h>
#include <utility/bash_cout.h>
#include <debug/cauv_debug.h>

#include <generated/types/TimeStamp.h>

#include "server_shared.h"

using namespace cauv;

/* How camera sharing works:
 *
 * A client program that wants access to a camera connects to a magic port
 * (16708) on localhost, and sends 4 bytes which are the camera id (unsigned
 * integer) that it wishes to have access to.
 *
 * This server sends this response (native byte order):
 * 
 *  [uint32_t lock offset] 0..7
 *  [uint32_t image offset] 8..15
 *  [uint32_t image width] 16..19
 *  [uint32_t image height] 20..23
 *  [uint32_t image type] 24..27 (OpenCV type, eg CV_8UC3)
 *  [char] 28..
 *  [char]
 *   ...
 *  [char]
 *  [  0 ] N  shared memory segment name
 *   ...
 *  [char]
 *  [  0 ] N  shared memory segment name
 *
 * Once it has received this information, the client can get an image:
 *
 *   1) set *(shared mem + lock offset) = 1
 *   2) Send 4 bytes over the socket. Value doesn't matter
 *   3) poll *(shared mem + lock offset), until the value changes to 2
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

class CameraServer;
class CameraManager;

class CameraClientConnection{
        const static uint32_t In_Buf_Size = 64;
    public:
        CameraClientConnection(CameraServer& server, boost::asio::io_service &io_service);

        boost::asio::ip::tcp::socket& socket();

        void start();

        void handleFirstRead(boost::system::error_code const& e, size_t s);
        void handleRead(boost::system::error_code const& e);
        void handleWrite(boost::system::error_code const& e);

        lock_ptr lockPtr() const;
        void setImageID(uint32_t image_id){m_image_id = image_id;}
        uint32_t imageID() const{return m_image_id;}

        void poll();

    private:
        void _setupFirstRead();
        void _setupRead();

        uint32_t m_camera_id;
        CameraManager *m_camera_manager;

        lock_ptr m_lock_ptr;
        uint32_t m_image_id;
        
        CameraServer &m_server;
        boost::asio::ip::tcp::socket m_socket;
        char m_data[In_Buf_Size];
        InfoResponse m_response_data;
};

struct CameraProperties{
    uint32_t id;
    uint32_t resize_to_width;
    uint32_t resize_to_height;
    int32_t convert_to_type;
};

class CameraManager: public cv::VideoCapture{
        typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
        const static uint32_t Max_Clients = 64;        
        const static uint32_t Max_Image_Latency_MSec = 50;
    public:
        CameraManager(CameraProperties const& cam_properties);
        ~CameraManager();
        std::string shareName() const;
        handle_t imageHandle() const;

        void putRequest(CameraClientConnection*);

        lock_ptr newLockPtr(CameraClientConnection*);
        void deleteLockPtr(lock_ptr, CameraClientConnection*);
        handle_t lockHandle(lock_ptr);

        CameraProperties const& camProps() const{ return m_cam_properties; }

    private:
        void _processPendingImageRequests();

        boost::interprocess::managed_shared_memory m_segment;
        uint8_t* m_image_ptr;
        handle_t m_image_handle;
        
        struct Request{
            CameraClientConnection *from;
            TimeStamp at;
        };
        std::list<Request> m_requests;
        //std::map<CameraClientConnection*, uint32_t*> m_lock_ptrs;

        TimeStamp m_last_image_time;
        uint32_t m_last_image_id;

        CameraProperties m_cam_properties;
};

class CameraServer{
        typedef boost::asio::ip::tcp tcp;
    public:
        CameraServer(boost::asio::io_service &service, uint32_t port);

        void doAccept(CameraClientConnection *c, boost::system::error_code const& e);

        CameraManager* didOpen(CameraClientConnection *c, uint32_t camera_id);
        void didClose(CameraClientConnection *c, uint32_t camera_id);

    private:
        void _setupPendingConnection();
        
        // open cameras, and the clients connected to them
        struct CamManager_CamClientConns{
            CameraManager* manager;
            std::set<CameraClientConnection*> connected;
        };
        std::map<uint32_t, CamManager_CamClientConns> m_cams_and_clients;

        boost::asio::io_service &m_service;
        tcp::acceptor m_acceptor;
};

// ======= Implementation ========

// CameraClientConnection

CameraClientConnection::CameraClientConnection(CameraServer& server, boost::asio::io_service &io_service)
    : m_camera_id(0),
      m_camera_manager(NULL),
      m_lock_ptr(NULL),
      m_server(server),
      m_socket(io_service){
}

boost::asio::ip::tcp::socket& CameraClientConnection::socket(){
    return m_socket;
}

void CameraClientConnection::start(){
    _setupFirstRead();
}

void CameraClientConnection::handleFirstRead(boost::system::error_code const& e, size_t s){
    if(!e){
        if(s == 4){
            m_camera_id = *((uint32_t*)m_data);
            m_camera_manager = m_server.didOpen(this, m_camera_id);
            m_lock_ptr = m_camera_manager->newLockPtr(this);
            // write response: 
            // TODO
            debug() << "\nlock handle:" << (uint64_t) m_camera_manager->lockHandle(m_lock_ptr)
                    << "\nimage handle:" << (uint64_t) m_camera_manager->imageHandle()
                    << "\nsegment:" << m_camera_manager->shareName();
            m_response_data.lock_offset = (uint64_t) m_camera_manager->lockHandle(m_lock_ptr);
            m_response_data.image_offset = (uint64_t) m_camera_manager->imageHandle();
            m_response_data.width = m_camera_manager->camProps().resize_to_width;
            m_response_data.height = m_camera_manager->camProps().resize_to_height;
            m_response_data.type = m_camera_manager->camProps().convert_to_type;
            std::string share_name = m_camera_manager->shareName();
            memset(m_response_data.name, 0, Max_InfoResponse_Name_Len);
            strncpy(m_response_data.name, share_name.c_str(), Max_InfoResponse_Name_Len);
            boost::asio::async_write(
                m_socket,
                boost::asio::buffer((uint8_t*)&m_response_data, sizeof(m_response_data)),
                boost::bind(
                    &CameraClientConnection::handleWrite, this,
                    boost::asio::placeholders::error
                )
            );
            _setupRead();
        }else{
            error() << "CameraClientConnection::handleFirstRead: client didn't send 4 bytes";
            delete this;
        }
    }else{
        debug() << "CameraClientConnection::handleFristRead failed: " << e << e.message();
        // this is (certainly?) because the remote client crashed or
        // otherwise disconnected
        delete this;
    }
}

void CameraClientConnection::handleWrite(boost::system::error_code const& e){
    if(e){
        debug() << "CameraClientConnection::handleWrite failed: " << e << e.message();
        // this is (certainly?) because the remote client crashed or
        // otherwise disconnected
        m_camera_manager->deleteLockPtr(m_lock_ptr, this);        
        m_server.didClose(this, m_camera_id);
    }
}

void CameraClientConnection::handleRead(boost::system::error_code const& e){
    if (!e){
        m_camera_manager->putRequest(this);
        _setupRead();
    }else{
        debug() << "CameraClientConnection::handleRead failed: " << e << e.message();
        // this is (certainly?) because the remote client crashed or
        // otherwise disconnected
        m_camera_manager->deleteLockPtr(m_lock_ptr, this);
        m_server.didClose(this, m_camera_id);
    }
}

lock_ptr CameraClientConnection::lockPtr() const{
    return m_lock_ptr;
}

void CameraClientConnection::poll(){
    boost::system::error_code e;
    m_socket.available(e);
    if(e){
        error() << e << e.message();
        m_camera_manager->deleteLockPtr(m_lock_ptr, this);        
        m_server.didClose(this, m_camera_id);
    }
}

void CameraClientConnection::_setupFirstRead(){
    m_socket.async_read_some(
        boost::asio::buffer(m_data, In_Buf_Size),
        boost::bind(
            &CameraClientConnection::handleFirstRead, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        )
    );
}

void CameraClientConnection::_setupRead(){
    m_socket.async_read_some(
        boost::asio::buffer(m_data, In_Buf_Size),
        boost::bind(
            &CameraClientConnection::handleRead, this,
            boost::asio::placeholders::error
        )
    );
}

// CameraManager
CameraManager::CameraManager(CameraProperties const& cam_properties)
    : cv::VideoCapture(),
      m_segment(),
      m_image_ptr(NULL),
      m_image_handle(),
      m_last_image_time(),
      m_last_image_id(0),
      m_cam_properties(cam_properties){
        // set up the shared memory for the camera image and locks:
        boost::interprocess::shared_memory_object::remove(shareName().c_str());
        uint32_t image_size = cam_properties.resize_to_width *
                              cam_properties.resize_to_height *
                              CV_ELEM_SIZE(cam_properties.convert_to_type);
        uint32_t segment_size = image_size + Max_Clients * 4;
        segment_size += 1024;
        segment_size -= (segment_size % 1024);
        m_segment = boost::interprocess::managed_shared_memory(
            boost::interprocess::create_only, shareName().c_str(), segment_size
        );
        m_image_ptr = (uint8_t*) m_segment.allocate(image_size);
        assert(m_image_ptr);
        m_image_handle = m_segment.get_handle_from_address(m_image_ptr);

        // set up the OpenCV capture (base class)
        // these don't always succeed, but if when we read a camera
        // image it isn't the right size, then we'll resize it then,
        // too:
        set(CV_CAP_PROP_FRAME_WIDTH, cam_properties.resize_to_width);
        set(CV_CAP_PROP_FRAME_HEIGHT, cam_properties.resize_to_height);
        open(cam_properties.id);
}
CameraManager::~CameraManager(){
    boost::interprocess::shared_memory_object::remove(shareName().c_str());
}

std::string CameraManager::shareName() const{
    return mkStr() << "Shared Camera " << m_cam_properties.id;
}

CameraManager::handle_t CameraManager::imageHandle() const{
    return m_image_handle;
}

void CameraManager::putRequest(CameraClientConnection* c){
    Request t = {c, now()};
    m_requests.push_back(t);
    _processPendingImageRequests();
}

lock_ptr CameraManager::newLockPtr(CameraClientConnection *c){
    lock_ptr r = (lock_ptr)m_segment.allocate(4);
    c=c;//assert(!m_lock_ptrs.count(c));
    //m_lock_ptrs[c] = r;
    return r;
}

void CameraManager::deleteLockPtr(lock_ptr p, CameraClientConnection *c){
    c=c;//assert(m_lock_ptrs[c] == p);
    m_segment.deallocate((void*)p);
    // eww, linear time:
    
    std::list<Request>::iterator i;
    for(i = m_requests.begin(); i != m_requests.end();){
        if(i->from == c)
            i = m_requests.erase(i);
        else
            i++;
    }
        
}

CameraManager::handle_t CameraManager::lockHandle(lock_ptr p){
    return m_segment.get_handle_from_address((void*)p);
}

static uint32_t msecdiff(TimeStamp const&a, TimeStamp const& b){
    return (a.secs - b.secs) * 1000 + (500 + a.musecs - b.musecs) / 1000;
}

void CameraManager::_processPendingImageRequests(){
    uint32_t blocked = 0;
    // remove finished requests:
    std::list<Request>::iterator i, j;
    for(i = m_requests.begin(); i != m_requests.end();)
        if(0 == *(i->from->lockPtr())){
            i = m_requests.erase(i);
        }else{
            ++i;
        }
    // TODO: implement lock release notification?
    do{
        if(blocked){
            msleep(1);
            // check that sockets are still ok (a client we're waiting for
            // hasn't crashed)
            for(i = m_requests.begin(); i != m_requests.end(); i++){
                j = i++;
                // may cause an item to be removed from m_requests (if a client
                // has closed)
                j->from->poll();
            }
        }
        blocked = 0;
        foreach(Request &r, m_requests){
            uint32_t lock_val = *(r.from->lockPtr());
            if(lock_val == 2)
                // already executing
                blocked++;
            else if(lock_val == 1){
                if(r.from->imageID() == m_last_image_id)
                    // same image as last time
                    blocked++; 
                else if(msecdiff(r.at, m_last_image_time) > Max_Image_Latency_MSec)
                    // image too old
                    blocked++;
            }
        }
    }while(blocked);
     
    cv::Mat shared_mat(
        m_cam_properties.resize_to_height,
        m_cam_properties.resize_to_width,
        m_cam_properties.convert_to_type,
        m_image_ptr
    );
    if(!grab()) error() << "failed to grab frame";
    m_last_image_time = now();
    cv::Mat internal_mat;
    if(!retrieve(internal_mat)) error() << "failed to retrieve frame";
    
    // There is no way to avoid this copy (without re-writing V4L), so we can
    // at least do something useful at the same time as copying:
    if(m_cam_properties.convert_to_type != internal_mat.type())
        error() << "converting type isn't supported yet";
    else
        cv::resize(
            internal_mat, shared_mat,
            cv::Size(m_cam_properties.resize_to_width,
                     m_cam_properties.resize_to_height)
        );
    
    m_last_image_id++;

    foreach(Request &r, m_requests){
        r.from->setImageID(m_last_image_id);
        *(r.from->lockPtr()) = 2;
    }
}


// CameraServer

CameraServer::CameraServer(boost::asio::io_service &service,
             uint32_t port)
    : m_cams_and_clients(),
      m_service(service),
      m_acceptor(service, tcp::endpoint(tcp::v4(), port)){

    // prepare a connection to accept a client              
    _setupPendingConnection();
}

void CameraServer::doAccept(CameraClientConnection *c, boost::system::error_code const& e){
    //debug() << "CameraServer::doAccept:" << c->socket().native() << e;
    if(!e){
        // start this connection, at some point later an asynchronous
        // call will (probably) be made back to didOpen()
        c->start();
        // prepare a new connection to accept another client
        _setupPendingConnection();
    }else{
        std::cerr << "CameraServer::doAccept failed: " << e.message() << std::endl;
    }
}

CameraManager* CameraServer::didOpen(CameraClientConnection *c, uint32_t camera_id){
    // !!! thread safety is not an issue because there is only one
    // worker thread for asio events
    std::map<uint32_t, CamManager_CamClientConns>::iterator i = m_cams_and_clients.find(camera_id);
    if(i == m_cams_and_clients.end()){
        CamManager_CamClientConns t;
        // !!! TODO: expose these via command line options, per camera id
        CameraProperties props = {
            camera_id, 640, 480, CV_8UC3
        };
        t.manager = new CameraManager(props);
        m_cams_and_clients[camera_id] = t;
        i = m_cams_and_clients.find(camera_id);
    }
    i->second.connected.insert(c);
    return i->second.manager;
}

void CameraServer::didClose(CameraClientConnection *c, uint32_t camera_id){
    // !!! thread safety is not an issue because there is only one
    // worker thread for asio events
    std::map<uint32_t, CamManager_CamClientConns>::iterator i = m_cams_and_clients.find(camera_id);
    std::set<CameraClientConnection*>::iterator j;
    if(i == m_cams_and_clients.end() || !i->second.connected.erase(c))
        error() << "unmatched didClose!";
    else if(!i->second.connected.size()){
        debug() << "camera" << camera_id << "is now unused";
        delete i->second.manager;
        m_cams_and_clients.erase(i);
    }
}

void CameraServer::_setupPendingConnection(){
    CameraClientConnection *next_connection = new CameraClientConnection(*this, m_service);
    m_acceptor.async_accept(
        next_connection->socket(),
        boost::bind(&CameraServer::doAccept, this, next_connection, boost::asio::placeholders::error)
    );
}



int main(int argc, char** argv){
    uint32_t port = 16708;
    try{
        boost::asio::io_service iosv;
        CameraServer s(iosv, port);
        iosv.run();
        debug() << "run loop finished";
    }catch(std::exception& e){
        error() << e.what();
    }
    return 0;
}
