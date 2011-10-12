#include "client.h"

#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

#include <opencv2/core/core.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_utils.h>

#include <generated/types/TimeStamp.h>

#include "server_shared.h"

using namespace cauv;

struct ImageWrapper: boost::noncopyable{
    ImageWrapper(SharedImage* s)
        : m_shared_image(s){
    }

    ~ImageWrapper(){
        m_shared_image->lock = 0;
    }

    cv::Mat mat(){
        return cv::Mat(
            m_shared_image->height,
            m_shared_image->width,
            m_shared_image->type,
            &(m_shared_image->bytes[0]),
            m_shared_image->pitch
        );
    }

    SharedImage* m_shared_image;
};

class CameraServerConnection{
    typedef boost::asio::ip::tcp tcp;
    typedef boost::interprocess::managed_shared_memory::handle_t handle_t;
    public:
        CameraServerConnection()
            : m_port(16708),
              m_service(),
              m_socket(),
              m_segment(){
            reInit();
        }
        
        boost::shared_ptr<ImageWrapper> getImage(uint32_t camera_id, uint32_t w, uint32_t h){
            if(!m_socket)
                reInit();

            ImageRequest req = {
                camera_id, w, h
            };
            boost::asio::write(*m_socket, boost::asio::buffer((uint8_t*)&req, sizeof(req)));

            InfoResponse resp;
            size_t reply_length = boost::asio::read(
                *m_socket, boost::asio::buffer(&resp, sizeof(resp))
            );

            debug(7) << "getImage handle:" << resp.image_offset;

            if(reply_length != sizeof(resp) || resp.image_offset == 0)
                throw std::runtime_error("could not get image");

            SharedImage *s = (SharedImage*) m_segment.get_address_from_handle(
                handle_t(resp.image_offset)
            );

            return boost::make_shared<ImageWrapper>(s);
        }

    private:
        void reInit(){
            m_socket = boost::make_shared<tcp::socket>(boost::ref(m_service));

            tcp::resolver r(m_service);
            tcp::resolver::query q(tcp::v4(), "localhost", mkStr() << m_port);
            tcp::resolver::iterator it = r.resolve(q);
            
            boost::asio::connect(*m_socket, it);
            
            m_segment = boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, SHMEM_SEGMENT_NAME
            );
        }

        uint32_t m_port;
        boost::asio::io_service m_service;
        boost::shared_ptr<tcp::socket> m_socket;

        boost::interprocess::managed_shared_memory m_segment;
};

int main(int argc, char** argv){
    try{
        boost::asio::io_service iosv;
        CameraServerConnection C;
        
        {
            {boost::shared_ptr<ImageWrapper> a = C.getImage(0, 640, 480);}
            {boost::shared_ptr<ImageWrapper> b = C.getImage(0, 640, 480);}
            boost::shared_ptr<ImageWrapper> c = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> d = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> e = C.getImage(0, 640, 480);
            {boost::shared_ptr<ImageWrapper> f = C.getImage(0, 640, 480);}
            boost::shared_ptr<ImageWrapper> g = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> h = C.getImage(0, 640, 480);
        }
        
        {
            {boost::shared_ptr<ImageWrapper> a = C.getImage(0, 640, 480);}
            {boost::shared_ptr<ImageWrapper> b = C.getImage(0, 320, 240);}
            {boost::shared_ptr<ImageWrapper> c = C.getImage(0, 100, 200);}
            {boost::shared_ptr<ImageWrapper> d = C.getImage(0, 100, 200);}
            boost::shared_ptr<ImageWrapper> e = C.getImage(0, 320, 240);
            boost::shared_ptr<ImageWrapper> f = C.getImage(0, 320, 240);
            boost::shared_ptr<ImageWrapper> g = C.getImage(0, 320, 240);
        }

        uint32_t N = 1000;
        TimeStamp tstart = now();
        for(uint32_t i = 0; i < N; i++)
            C.getImage(0, 640, 480);
        TimeStamp tend = now();
        uint64_t musecdiff = tend.musecs - tstart.musecs + 1000000*(tend.secs - tstart.secs);
        debug() << (1e6 * N) / musecdiff << "fps";

    }catch(std::exception& e){
        error() << e.what();
    }
    return 0;
}
