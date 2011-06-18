#ifndef __THROTTLE_NODE_H__
#define __THROTTLE_NODE_H__

#include "../node.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <common/cauv_utils.h>

namespace cauv{
namespace imgproc{

class ThrottleNode: public Node{
    private:
        typedef boost::mutex mutex_t;
        typedef boost::unique_lock<mutex_t> unique_lock_t;

    public:
        ThrottleNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t),
              m_mux(),
              m_need_callback(),
              m_will_be_destroyed(false),
              m_ioservice(),
              m_current_timer_rate(5),
              m_timer_ptr(boost::make_shared<boost::asio::deadline_timer>(boost::ref(m_ioservice))),
              m_ioservice_thread(boost::bind(&ThrottleNode::runIOService, this)){
        }

        void init(){
            m_speed = asynchronous;

            // input:
            registerInputID(Image_In_Name);

            // outputs:
            registerOutputID<image_ptr_t>(Image_Out_Name);
            registerOutputID<NodeParamValue>("actual frequency");

            // rate parameter (Hz)
            registerParamID<float>("target frequency", 20, "maximum frequency (Hz)");
            paramChanged("target frequency");

            m_last_exec = now();
        }

        virtual ~ThrottleNode(){
            unique_lock_t l(m_mux);
            m_will_be_destroyed = true;
            m_timer_ptr->cancel();
            l.unlock();
            while(m_will_be_destroyed){
                unique_lock_t l(m_mux);
                m_need_callback.notify_one();
            }

            m_ioservice_thread.join();
            stop();
        }

        virtual void paramChanged(param_id const& p){
            if(p != "target frequency")
                return;

            float timer_rate = param<float>("target frequency");

            // if you're ever want to set a timer for more than 1 billion
            // seconds, get back to me.
            if(timer_rate < 1e-9){
                timer_rate = 1e-9;
                warning() << "clamping throttle rate to 1e-9 Hz";
            }

            unique_lock_t l(m_mux);
            if(timer_rate != m_current_timer_rate){
                m_current_timer_rate = timer_rate;
                l.unlock();
                _setupCallback();
            }
        }

        void runIOService(){
            boost::system::error_code ec; // default-initialised to 0 (success)
            std::size_t executed = 1;
            unsigned no_exec_count = 0;
            /**
             * Simplifications to this loop are welcome but please test them
             * first. Writing this node didn't take a day longer than it should
             * have for no reason...
             */
            while(no_exec_count < 2){
                unique_lock_t l(m_mux);
                if(executed)
                    m_need_callback.wait(l);
                if(m_will_be_destroyed){
                    m_will_be_destroyed = false;
                    break;
                 }
                l.unlock();
                if(executed == 0 || ec)
                    m_ioservice.reset();
                executed = m_ioservice.run(ec);
                if(!executed)
                    no_exec_count++;
                else
                    no_exec_count = 0;
                debug(4) << "ThrottleNode: executed" << executed << ", status" << ec;
            }
            debug() << "ThrottleNode callback thread stopped";
        }

        void timerCallback(const boost::system::error_code& err){
            if(err == boost::system::errc::success)
                demandNewParentInput();
            else if(err == boost::asio::error::operation_aborted)
                warning() << "ThrottleNode callback aborted";
            else
                error() << "ThrottleNode callback error:" << err;
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            r[Image_Out_Name] = inputs[Image_In_Name];

            TimeStamp n = now();
            double tdiff = (n.secs - m_last_exec.secs) + (n.musecs - m_last_exec.musecs)/1e6;
            if(tdiff > 0)
                r["actual frequency"] = NodeParamValue(float(1.0/tdiff));
            else
                r["actual frequency"] = NodeParamValue(0.0f);

            m_last_exec = now();

            _setupCallback();

            return r;
        }

    private:
        void _setupCallback(){
            // have to reschedule from the same thread
            unique_lock_t l(m_mux);
            boost::system::error_code ec;
            m_timer_ptr->expires_from_now(boost::posix_time::microseconds(1.0e6f/m_current_timer_rate), ec);
            if(ec)
                error() << "setting timer expiration:" << ec;
            m_timer_ptr->async_wait(boost::bind(&ThrottleNode::timerCallback, this, _1));
            m_need_callback.notify_one();
        }

        mutex_t m_mux;
        boost::condition_variable m_need_callback;
        volatile bool m_will_be_destroyed;

        boost::asio::io_service m_ioservice;
        float m_current_timer_rate;
        TimeStamp m_last_exec;
        boost::shared_ptr<boost::asio::deadline_timer> m_timer_ptr;
        boost::thread m_ioservice_thread;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __THROTTLE_NODE_H__
