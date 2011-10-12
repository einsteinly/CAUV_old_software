#ifndef __CAUV_CAMERA_CLIENT_H__
#define __CAUV_CAMERA_CLIENT_H__

#include <boost/asio.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

namespace cauv{

struct SharedImage;

struct ImageWrapper: boost::noncopyable{
    ImageWrapper(SharedImage* s);
    ~ImageWrapper();
    cv::Mat mat();

    SharedImage* m_shared_image;
};

class CameraServerConnection{
    typedef boost::asio::ip::tcp tcp;
    typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
    public:
        CameraServerConnection();
        boost::shared_ptr<ImageWrapper> getImage(uint32_t camera_id, uint32_t w, uint32_t h);
        SharedImage* getUnGuardedImage(uint32_t camera_id, uint32_t w, uint32_t h);

    private:
        void reInit();

        uint32_t m_port;
        boost::asio::io_service m_service;
        boost::shared_ptr<tcp::socket> m_socket;

        boost::interprocess::managed_shared_memory m_segment;
};


} // namespace cauv

#endif // ndef __CAUV_CAMERA_CLIENT_H__


