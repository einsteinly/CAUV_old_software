#include "client.h"

#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <opencv2/core/core.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_utils.h>

#include "server_shared.h"

using namespace cauv;

class CameraServerConnection{
    typedef boost::asio::ip::tcp tcp;
    typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
    public:
        CameraServerConnection(uint32_t camera_id)
            : m_port(16708),
              m_service(),
              m_socket(),
              m_camera_id(camera_id),
              m_segment(),
              m_lock_ptr(0),
              m_image_ptr(0)
            {
            reInit();
        }
        
        // Call finishedWithImage after each call to this. The returned image
        // is in shared memory, do not modify it in place. 
        cv::Mat getImage(){
            if(!m_socket)
                reInit();
            cv::Mat shared_mat(
                m_info_reply.height,
                m_info_reply.width,
                m_info_reply.type,
                m_image_ptr
            );
            try{
                *m_lock_ptr = 1;
                boost::asio::write(*m_socket, boost::asio::buffer((uint8_t*)&m_camera_id, 4));
                while (*m_lock_ptr != 2){
                    debug() << "no image yet" << *m_lock_ptr;
                    msleep(0);
                }
                return shared_mat;
            }catch(std::exception& e){
                error() << "failed to get image:" << e.what();
                m_lock_ptr = 0;
                m_image_ptr = 0;
                m_socket.reset();
            }
            return cv::Mat();
        }

        void finishedWithImage(){
            *m_lock_ptr = 0;
        }

    private:
        void reInit(){
            m_socket = boost::make_shared<tcp::socket>(boost::ref(m_service));

            tcp::resolver r(m_service);
            tcp::resolver::query q(tcp::v4(), "localhost", mkStr() << m_port);
            tcp::resolver::iterator it = r.resolve(q);
            
            boost::asio::connect(*m_socket, it);
            
            debug() << "sending camera id...";
            boost::asio::write(*m_socket, boost::asio::buffer((uint8_t*)&m_camera_id, 4));
            
            debug() << "reading reply...";
            size_t reply_length = boost::asio::read(
                *m_socket,
                boost::asio::buffer(&m_info_reply, sizeof(m_info_reply))
            );
            debug() << "      reInit: RX" << reply_length;
            debug() << " lock offset:" << m_info_reply.lock_offset;
            debug() << "image offset:" << m_info_reply.image_offset;
            debug() << "        name:" << m_info_reply.name;
            debug() << "       width:" << m_info_reply.width;
            debug() << "      height:" << m_info_reply.height;
            

            m_segment = boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, m_info_reply.name
            );
            handle_t lock_handle = m_info_reply.lock_offset;
            handle_t image_handle = m_info_reply.image_offset;

            m_lock_ptr = (lock_ptr) m_segment.get_address_from_handle(lock_handle);
            m_image_ptr = (uint8_t*) m_segment.get_address_from_handle(image_handle);
        }

        uint32_t m_port;
        boost::asio::io_service m_service;
        boost::shared_ptr<tcp::socket> m_socket;
        uint32_t m_camera_id;

        boost::interprocess::managed_shared_memory m_segment;
        lock_ptr m_lock_ptr;
        uint8_t *m_image_ptr;

        InfoResponse m_info_reply;

};

int main(int argc, char** argv){
    try{
        boost::asio::io_service iosv;
        CameraServerConnection c(1);

        c.getImage();
        c.finishedWithImage();

    }catch(std::exception& e){
        error() << e.what();
    }
    return 0;
}
