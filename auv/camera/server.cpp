#include "server.h"

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <common/cauv_utils.h>
#include <utility/bash_cout.h>
#include <debug/cauv_debug.h>

using namespace cauv;

// CameraClientConnection

CameraClientConnection::CameraClientConnection(CameraServer& server,
                                               CameraManager& manager,
                                               boost::asio::io_service &io_service)
    : m_camera_manager(manager),
      m_current_images(),
      m_server(server),
      m_socket(io_service),
      m_temp_response_data(){
}

CameraClientConnection::~CameraClientConnection(){
    m_camera_manager.release(m_current_images);
    m_camera_manager.didClose(this);
}

tcp::socket& CameraClientConnection::socket(){
    return m_socket;
}

void CameraClientConnection::start(){
    _setupRead();
}

void CameraClientConnection::handleWrite(bs::error_code const& e){
    if(!e){
        _setupRead();
    }else{
        debug() << "CameraClientConnection::handleWrite failed: " << e << e.message();
        // destructor cleans up everything
        delete this;
    }
}

void CameraClientConnection::handleRead(bs::error_code const& e, size_t s){
    if (!e && s == sizeof(m_temp_image_request)){
        m_camera_manager.getImage(
            m_temp_response_data, m_current_images, m_temp_image_request, this
        );
        boost::asio::async_write(
            m_socket,
            boost::asio::buffer((uint8_t*)&m_temp_response_data, sizeof(m_temp_response_data)),
            boost::bind(
                &CameraClientConnection::handleWrite, this,
                boost::asio::placeholders::error
            )
        );
    }else{
        debug() << "CameraClientConnection::handleRead failed: " << e << e.message();
        // destructor cleans up everything
        delete this;
    }
}

void CameraClientConnection::_setupRead(){
    m_socket.async_read_some(
        boost::asio::buffer((uint8_t*)&m_temp_image_request, sizeof(m_temp_image_request)),
        boost::bind(
            &CameraClientConnection::handleRead, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        )
    );
}

// CameraManager
CameraManager::CameraManager()
    : m_segment(),
      m_open_cameras(),
      m_camera_users(){
    // set up the shared memory for the camera image and locks:
    boost::interprocess::shared_memory_object::remove(SHMEM_SEGMENT_NAME);
    // TODO: this shouldn't be hard-coded here! we're only using it to
    // calculate the size to grab for shared mem though
    uint32_t image_size = 640 * 480 * 3 + sizeof(SharedImage);
    image_size += 1024;
    image_size -= (image_size % 1024);
    uint32_t segment_size = image_size * Max_Concurrent_Images;
    m_segment = boost::interprocess::managed_shared_memory(
        boost::interprocess::create_only, SHMEM_SEGMENT_NAME, segment_size
    );
}
CameraManager::~CameraManager(){
    boost::interprocess::shared_memory_object::remove(SHMEM_SEGMENT_NAME);
}

void CameraManager::getImage(InfoResponse& resp,
                             std::list<uint64_t>& active_images,
                             ImageRequest const& req,
                             CameraClientConnection *c){
    // check we have the requested camera open:
    if(!m_open_cameras.count(req.camera_id)){
        // !!! TODO: in some circumstances OpenCV has been known to open a
        // different camera id to the one requested (if the requested ID was
        // too high, we need to handle that case here!)
        m_open_cameras[req.camera_id] = boost::make_shared<Capture>(req.camera_id);
    }

    boost::shared_ptr<Capture> cap = m_open_cameras[req.camera_id];

    int32_t cap_type = CV_8UC3;

    // release any images that this client isn't using any more, and possibly
    // allocate in place
    bool did_allocate_in_place = false;
    std::list<uint64_t>::iterator i;
    for(i = active_images.begin(); i != active_images.end();){
        SharedImage* s = (SharedImage*) m_segment.get_address_from_handle(*i);
        if(!s->lock){
            if(!did_allocate_in_place &&
               s->width == req.w &&
               s->height == req.h &&
               s->type == cap_type){
                std::cout << BashColour::Brown << "=" << BashColour::None << std::flush;
                did_allocate_in_place = true;
                cap->captureToMem(&(s->bytes[0]), s->pitch, req.w, req.h, cap_type);
                resp.image_offset = m_segment.get_handle_from_address((void*)s);
                i++;
            }else{
                std::cout << BashColour::Red << "-" << BashColour::None << std::flush;
                m_segment.deallocate(s);
                i = active_images.erase(i);
            }
        }else
            i++;
    }

    if(!did_allocate_in_place){
        uint32_t alloc_size = offsetof(SharedImage, bytes) + (req.w+4)*req.h*3; 
        try{
            SharedImage *r = (SharedImage*) m_segment.allocate(alloc_size);
            std::cout << BashColour::Green << "+" << BashColour::None << std::flush;
            r->width = req.w;
            r->height = req.h;
            r->type = CV_8UC3;
            r->lock = 1;
            cap->captureToMem(&(r->bytes[0]), r->pitch, req.w, req.h, cap_type);
            resp.image_offset = m_segment.get_handle_from_address((void*)r);
            active_images.push_back(resp.image_offset);
        }catch(boost::interprocess::bad_alloc &e){
            error() << "could not allocate image!" << e.what();
            // !!! TODO: not sure if 0-offset is a valid allocation address,
            // check this, and if so use -1 or something to indicate error
            resp.image_offset = 0;
        }
    }

    m_camera_users[req.camera_id].insert(c);
}

void CameraManager::didClose(CameraClientConnection *c){
    std::vector<uint32_t> cameras_for_closing;
    std::map<uint32_t, std::set<CameraClientConnection*> >::iterator i;
    for(i = m_camera_users.begin(); i != m_camera_users.end(); i++){
        i->second.erase(c);
        if(!i->second.size())
            cameras_for_closing.push_back(i->first);
    }
    foreach(uint32_t c, cameras_for_closing){
        debug() << "camera" << c << "is now unused";
        m_open_cameras.erase(c);
        m_camera_users.erase(c);
    }
}

void CameraManager::release(std::list<uint64_t> const& images){
    foreach(uint64_t const& i, images){
        std::cout << BashColour::Red << "-" << BashColour::None << std::flush;
        m_segment.deallocate(m_segment.get_address_from_handle(handle_t(i)));
    }
}

// CameraManager::Capture
CameraManager::Capture::Capture(uint32_t cam_id)
    : cv::VideoCapture(cam_id){
}

void CameraManager::Capture::captureToMem(
    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
){
    cv::Mat shared_mat(w, h, type, p);
    if(!grab())
        error() << "failed to grab frame";
    cv::Mat internal_mat;
    if(!retrieve(internal_mat))
        error() << "failed to retrieve frame";
    
    // There is no way to avoid this copy (without re-writing V4L), so we can
    // at least do something useful at the same time as copying:
    if(type != internal_mat.type())
        error() << "converting type isn't supported yet";
    else
        cv::resize(
            internal_mat, shared_mat,
            cv::Size(w, h)
        );
    pitch = shared_mat.step;
}

// CameraServer

CameraServer::CameraServer(boost::asio::io_service &service,
             uint32_t port)
    : m_manager(),
      m_service(service),
      m_acceptor(service, tcp::endpoint(tcp::v4(), port)){

    _setupPendingConnection();
}

void CameraServer::doAccept(CameraClientConnection *c, bs::error_code const& e){
    //debug() << "CameraServer::doAccept:" << c->socket().native() << e;
    if(!e){
        c->start();
        _setupPendingConnection();
    }else{
        std::cerr << "CameraServer::doAccept failed: " << e.message() << std::endl;
    }
}

void CameraServer::_setupPendingConnection(){
    CameraClientConnection *next_connection = new CameraClientConnection(*this, m_manager, m_service);
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
