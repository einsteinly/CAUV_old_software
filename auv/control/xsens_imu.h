#ifndef _XSENS_IMU_H
#define	_XSENS_IMU_H

#include <boost/thread.hpp>

#include <xsens/cmt3.h>

#include <common/messages.h>

class XsensObserver
{
    public:
        virtual void onTelemetry(const floatYPR& attitude) = 0;
};

class XsensIMU : public Observable<XsensObserver>
{
    public:
        XsensIMU(int id);
        XsensIMU(const XsensIMU& orig);
        virtual ~XsensIMU();

        floatYPR getAttitude();
        void configure(CmtOutputMode &mode, CmtOutputSettings &settings);
        void setObjectAlignmentMatrix(CmtMatrix m);

        void start();

    protected:
        xsens::Cmt3 m_cmt3;
        CmtPortInfo m_port;
        boost::thread m_readThread;

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

#endif	/* _XSENS_IMU_H */

