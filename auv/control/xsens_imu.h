/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_XSENS_IMU_H__
#define __CAUV_XSENS_IMU_H__

#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>
#include <generated/message_observers.h>

#include <xsens/cmt3.h>

#include <utility/observable.h>

#include "imu.h"

namespace cauv{

class XsensIMU : public IMU, public MessageObserver
{
    public:
        XsensIMU(int id);
        virtual ~XsensIMU();

        void configure(CmtOutputMode &mode, CmtOutputSettings &settings);
        void calibrateNoRotation(uint16_t duration);
        void setObjectAlignmentMatrix(CmtMatrix m);

        void start();

        virtual void onCalibrateNoRotationMessage(CalibrateNoRotationMessage_ptr m);
    protected:
        xsens::Cmt3 m_cmt3;
        CmtPortInfo m_port;
        boost::thread m_readThread;
        boost::mutex m_cmt3_lock;
        bool m_running_norotation;

        void readThread();
};


class XsensException : public std::exception
{
    protected:
        std::string message;
    public:
        XsensException(const std::string& msg);
        ~XsensException() throw();
        virtual const char* what() const throw();
};

} // namespace cauv

#endif // ndef __CAUV_XSENS_IMU_H__

