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

#include "client.h"

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_utils.h>

#include <generated/types/TimeStamp.h>

#include "server_shared.h"

using namespace cauv;

ImageWrapper::ImageWrapper(SharedImage* s)
    : m_shared_image(s){
}

ImageWrapper::~ImageWrapper(){
    m_shared_image->lock = 0;
}

cv::Mat ImageWrapper::mat(){
    return cv::Mat(
        m_shared_image->height,
        m_shared_image->width,
        m_shared_image->type,
        &(m_shared_image->bytes[0]),
        m_shared_image->pitch
    );
}


CameraServerConnection::CameraServerConnection()
    : m_port(CAMERA_SERVER_PORT),
      m_service(),
      m_socket(),
      m_segment(){
    reInit();
}

boost::shared_ptr<ImageWrapper> CameraServerConnection::getImage(uint32_t camera_id, uint32_t w, uint32_t h){
    return boost::make_shared<ImageWrapper>(getUnGuardedImage(camera_id, w, h));
}

SharedImage* CameraServerConnection::getUnGuardedImage(uint32_t camera_id, uint32_t w, uint32_t h){
    if(!m_socket)
        reInit();
    if(!m_socket)
        throw std::runtime_error("could not connect to camera server");

    ImageRequest req = {
        camera_id, w, h
    };
    boost::asio::write(*m_socket, boost::asio::buffer((uint8_t*)&req, sizeof(req)));

    InfoResponse resp;
    size_t reply_length = boost::asio::read(
        *m_socket, boost::asio::buffer(&resp, sizeof(resp))
    );

    if(reply_length != sizeof(resp) || resp.image_offset == 0)
        throw std::runtime_error("could not get image");

    SharedImage *s = (SharedImage*) m_segment.get_address_from_handle(
        handle_t(resp.image_offset)
    );

    debug(8) << "w" << s->width
            << "h" << s->height
            << "type" << s->type
            << "pitch" << s->pitch
            << "bytes" << (void*) &(s->bytes[0]);

    return s;
}

void CameraServerConnection::reInit(){
    m_socket = boost::make_shared<tcp::socket>(boost::ref(m_service));

    tcp::resolver r(m_service);
    tcp::resolver::query q(tcp::v4(), "localhost", mkStr() << m_port);
    tcp::resolver::iterator it = r.resolve(q);
    boost::system::error_code e;
    m_socket->connect(*it, e);
    if(e){
        error() << "Could not connect socket:" << e;
        m_socket.reset();
        return;
    }
    
    m_segment = boost::interprocess::managed_shared_memory(
        boost::interprocess::open_only, SHMEM_SEGMENT_NAME
    );
}
