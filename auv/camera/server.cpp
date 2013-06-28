/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "server.h"

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <utility/bash_cout.h>
#include <utility/foreach.h>
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
        m_open_cameras[req.camera_id] = openCam(req);
    }

    boost::shared_ptr<Capture> cap = m_open_cameras[req.camera_id];

    if(!cap->ok()){
        std::cout << BashColour::Red << "!";
        debug() << "capture not available!";
        m_open_cameras.erase(req.camera_id);
        return;
    }

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
                s->lock = 1;
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
            r->type = cap_type;
            r->lock = 1;
            cap->captureToMem(&(r->bytes[0]), r->pitch, req.w, req.h, cap_type);
            resp.image_offset = m_segment.get_handle_from_address((void*)r);
            active_images.push_front(resp.image_offset);
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

boost::shared_ptr<CameraManager::Capture> CameraManager::openCam(ImageRequest const& req){
    #ifdef CAUV_USE_DC1394
    if(req.camera_id < 0)
        return boost::make_shared<DC1394Capture>(boost::ref(req));
    #endif //def CAUV_USE_DC1394
    return boost::make_shared<CVCapture>(req.camera_id);
}

// CameraManager::CVCapture
CameraManager::CVCapture::CVCapture(int32_t cam_id)
    : cv::VideoCapture(cam_id){
}

void CameraManager::CVCapture::captureToMem(
    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
){
    cv::Mat shared_mat(h, w, type, p);
    if(!grab())
        error() << "failed to grab frame";
    cv::Mat internal_mat;
    if(!retrieve(internal_mat))
        error() << "failed to retrieve frame";

    // There is no way to avoid this copy (without re-writing V4L), so we can
    // at least do something useful at the same time as copying:
    if(type != internal_mat.type()){
        // TODO: support Bayer conversion (somehow recognise bayer cameras??)
        cv::Mat tmp;
        if(int32_t(w) == internal_mat.cols && int32_t(h) == internal_mat.rows)
            tmp.convertTo(shared_mat, type);
        else{
            cv::resize(internal_mat, tmp, cv::Size(w, h));
            tmp.convertTo(shared_mat, type);
        }
    }else{
        if(int32_t(w) == internal_mat.cols && int32_t(h) == internal_mat.rows)
            internal_mat.copyTo(shared_mat);
        else
            cv::resize(internal_mat, shared_mat, cv::Size(w, h));
    }
    pitch = shared_mat.step;

    debug(8) << "w" << w << "h" << h << "type" << type << "pitch" << pitch << "bytes" << (void*)p;
}

bool CameraManager::CVCapture::ok() const{
    return isOpened();
}

#ifdef CAUV_USE_DC1394
// CameraManager::DC1394Capture
CameraManager::DC1394Capture::DC1394Capture(ImageRequest const& req)
    : m_dc1394(NULL),
      m_camera(NULL),
      m_video_mode(){
    dc1394camera_list_t* list;
    //dc1394video_modes_t video_modes;    
    dc1394error_t err;
    int32_t cam_id = -(1+req.camera_id);

    m_dc1394 = dc1394_new();
    if(!m_dc1394)
        throw std::runtime_error("failed to alloc dc1394");

    err=dc1394_camera_enumerate(m_dc1394, &list);
    if(err){
        error() << "could not enumerate dc1394 cameras";
        return;
    }

    if(cam_id >= int(list->num) || cam_id < 0){
        error() << "dc1394 camera" << cam_id << "is not available."
                << list->num << "1394 cameras are available.";
        return;
    }
    
    m_camera = dc1394_camera_new(m_dc1394, list->ids[cam_id].guid);
    if(!m_camera)
        throw std::runtime_error("failed to alloc dc1394 camera");
    dc1394_camera_free_list(list);
    
    debug() << "dc1394 cam GUID:" << m_camera->guid;
    
    // tmp: print all cam features
    dc1394featureset_t features;    
    err=dc1394_feature_get_all(m_camera,&features);
    if (err!=DC1394_SUCCESS) {
        dc1394_log_warning("Could not get feature set");
    }
    else {
        dc1394_feature_print_all(&features, stdout);
    }
    
    // set camera properties

    // ISO speed is firewire bus speed: from the dc1394 documentation:
    // ((
    //      Most (if not all) cameras are compatible with 400Mbps speed. Only
    //      older cameras (pre-1999) may still only work at sub-400 speeds.
    //      However, speeds lower than 400Mbps are still useful: they can be
    //      used for longer distances (e.g. 10m cables). Speeds over 400Mbps
    //      are only available in "B" mode (DC1394_OPERATION_MODE_1394B).
    //                                                                    ))
    err=dc1394_video_set_iso_speed(m_camera, DC1394_ISO_SPEED_400);
    if(err){
        error() << "failed to set bandwidth";
        return;
    }

    // supported video modes & framerates:
    //err=dc1394_video_get_supported_modes(m_camera,&video_modes);
    // ... m_video_mode = video_modes.mode[x]
    //err=dc1394_video_get_supported_framerates(m_camera,m_video_mode,&framerates);

    err=dc1394_video_set_mode(m_camera, DC1394_VIDEO_MODE_640x480_MONO8);
    if(err){
        error() << "failed to set video mode";
        return;
    }

    err=dc1394_video_set_framerate(m_camera, DC1394_FRAMERATE_7_5);
    if(err){
        error() << "failed to set framerate";
        return;
    }

    err=dc1394_capture_setup(m_camera,4, DC1394_CAPTURE_FLAGS_DEFAULT);
    if(err){
        error() << "could not setup camera";
        return;
    }

    err=dc1394_video_get_mode(m_camera, &m_video_mode);
    if(err){
        error() << "could not get video mode";
        return;
    }

    // start transmission
    err=dc1394_video_set_transmission(m_camera, DC1394_ON);
    if(err){
        error() << "could not start dc1394 transmission:" << err;
        return;
    }
}

CameraManager::DC1394Capture::~DC1394Capture(){
    if(m_camera){
        dc1394_video_set_transmission(m_camera, DC1394_OFF);
        dc1394_capture_stop(m_camera);
        dc1394_camera_free(m_camera);
        dc1394_free(m_dc1394);
    }
}


void CameraManager::DC1394Capture::captureToMem(
    uint8_t *p, uint32_t& pitch, uint32_t w, uint32_t h, int32_t type
){
    cv::Mat shared_mat(h, w, type, p);
    dc1394video_frame_t *frame;
    dc1394error_t err;    
    uint32_t width;
    uint32_t height;

    // get a pointer to a frame in the buffer
    err=dc1394_capture_dequeue(m_camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    if(err){
        error() << "could not grab frame" << err;
        return;
    }
    dc1394_get_image_size_from_video_mode(m_camera, m_video_mode, &width, &height);
    
    // super-hacky bit
    // !!! TODO: key the bayer conversion off actual information from the
    // camera... 
    cv::Mat internal_mat(height, width, CV_8UC1, frame->image);
    
    if(type == CV_8UC3){
        if(height==h && width==w){
            cv::cvtColor(internal_mat, shared_mat, cv::COLOR_BayerBG2BGR, 3);
        }else{
            cv::Mat tmp;
            cv::cvtColor(internal_mat, tmp, cv::COLOR_BayerRG2RGB, 3);
            cv::resize(tmp, shared_mat, cv::Size(w, h)); 
        }
    }else{
        if(w == width && h == height)    
            internal_mat.copyTo(shared_mat);
        else
            cv::resize(internal_mat, shared_mat, cv::Size(w, h)); 
    }
    // end super-hacky bit

    pitch = shared_mat.step;
    
    // return buffer memory
    dc1394_capture_enqueue(m_camera, frame);

    debug(8) << "w" << w << "h" << h << "type" << type << "pitch" << pitch << "bytes" << (void*)p;
}

bool CameraManager::DC1394Capture::ok() const{
    return m_camera && m_dc1394;
}

#endif // def CAUV_USE_DC1394

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



int main(int, char**){
    try{
        boost::asio::io_service iosv;
        CameraServer s(iosv, CAMERA_SERVER_PORT);
        iosv.run();
        debug() << "run loop finished";
    }catch(std::exception& e){
        error() << e.what();
    }
    return 0;
}
