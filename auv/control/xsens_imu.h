/* 
 * File:   xsens_imu.h
 * Author: andy
 *
 * Created on 27 June 2009, 17:14
 */

#ifndef _XSENS_IMU_H
#define	_XSENS_IMU_H

#include <lib/xsens/cmt3.h>

#include <common/cauv_types.h>

class XsensIMU
{
    public:
        XsensIMU(int id);
        XsensIMU(const XsensIMU& orig);
        virtual ~XsensIMU();

        floatYPR getAttitude();
        void configure(CmtOutputMode &mode, CmtOutputSettings &settings);
        void setObjectAlignmentMatrix(CmtMatrix m);

    private:
        xsens::Cmt3 m_cmt3;
        CmtPortInfo m_port;
};


class XsensException : public exception
{
    protected:
        string message;
    public:
        XsensException(const string& msg);
        ~XsensException() throw();
        virtual const char* what() const throw();
};

#endif	/* _XSENS_IMU_H */

