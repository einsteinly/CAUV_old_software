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

//#ifndef __CAUV_XSENS_IMU_H__
//#define	__CAUV_XSENS_IMU_H__
#ifndef __CAUV_SBG_IMU_H__
#define __CAUV_SBG_IMU_H__


#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

//#include <xsens/cmt3.h>
#include "sbgCom/sbgCom.h"

#include <utility/observable.h>

#include "imu.h"

namespace cauv{

//class XsensIMU : public Observable<IMUObserver>, boost::noncopyable
class sbgIMU : public IMU
{
    public:
        //XsensIMU(int id);
		sbgIMU(const char* port, int baud_rate, int pause_time);
        //virtual ~XsensIMU();
		virtual ~sbgIMU();

        //void configure(CmtOutputMode &mode, CmtOutputSettings &settings);
        //void calibrateNoRotation(uint16_t duration);
        //void setObjectAlignmentMatrix(CmtMatrix m);

        void start();

	private:
		
		const char* m_port;
	    int         m_baud_rate;
	    int         m_pause_time;

		SbgProtocolHandle protocolHandle;
		SbgErrorCode error;
		SbgOutput output;

    protected:
        //xsens::Cmt3 m_cmt3;
		//CmtPortInfo m_port;
        boost::thread m_readThread;
        boost::mutex m_cmt3_lock;
        bool m_running_norotation;

        void readThread();



};


//class XsensException : public std::exception
class sbgException : public std::exception
{
    protected:
        std::string message;
    public:
        //XsensException(const std::string& msg);
		sbgException(const std::string& msg);
		//~XsensException() throw();
		~sbgException() throw();
        virtual const char* what() const throw();
};

} // namespace cauv

#endif // ndef __CAUV_SBG_IMU_H__

