#ifndef __MODULE_H
#define __MODULE_H__

#include <ftdi.h>
#include <iostream>
#include <string.h>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/thread.hpp>

#include <common/messages.h>

class FTDIException : public std::exception
{
    protected:
        std::string message;
    public:
        FTDIException(const std::string& msg);
        FTDIException(const std::string& msg, int errCode, ftdi_context* ftdic);
        ~FTDIException() throw();
        virtual const char* what() const throw();
};

class FTDIStream : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        FTDIStream() throw();
        FTDIStream(int deviceID) throw(FTDIException);
    
        void baudrate(int baudrate) throw(FTDIException);
        void lineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity) throw(FTDIException);
        void flowControl(int flowControl) throw(FTDIException);

        std::streamsize read(char* s, std::streamsize n);
        std::streamsize write(const char* s, std::streamsize n);
        void close();
        void close(std::ios_base::openmode which);

    protected:
        struct ftdi_context ftdic;
};

class Module : public MessageSource
{
	public:
        ~Module();
        void send(Message& message) throw(FTDIException);
    
    protected:
        FTDIStream ftdiStream;
        boost::shared_ptr<boost::thread> m_readThread;
        volatile bool m_running;

        Module(int deviceID, int baudrate, ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity, int flowControl) throw(FTDIException);
    
        void readLoop();
};

void readDevice(Module* module);

#endif // __MODULE_H__
