#ifndef __MODULE_H
#define __MODULE_H__

#include <ftdi.h>
#include <iostream>
#include <string.h>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/thread.hpp>
//#include "module_message.h"

using namespace std;

class FTDIException : public exception
{
    protected:
        string message;
    public:
        FTDIException(const string& msg);
        FTDIException(const string& msg, int errCode, ftdi_context* ftdic);
        ~FTDIException() throw();
        virtual const char* what() const throw();
};

class FTDISource : public boost::iostreams::source
{
    public:
        FTDISource() throw();
        FTDISource(int deviceID) throw(FTDIException);
    
        void baudrate(int baudrate) throw(FTDIException);
        void lineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity) throw(FTDIException);
        void flowControl(int flowControl) throw(FTDIException);

        std::streamsize read(char* s, std::streamsize n);
        void close();

    protected:
        struct ftdi_context ftdic;
};

class Module
{
	public:
        ~Module();
        size_t read(u_char* buf, size_t buflen) throw(FTDIException);
//      virtual int handleData(ModuleData& data);
    //    int send(ModulePacket& packet);
      //  int send(ModuleMessage& message);
    protected:
        FTDISource ftdiSource;
        boost::shared_ptr<boost::thread> m_readThread;
        volatile bool m_running;

        Module(int deviceID, int baudrate, ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity, int flowControl) throw(FTDIException);
    
        void readLoop();
};

void readDevice(Module* module);

#endif // __MODULE_H__
