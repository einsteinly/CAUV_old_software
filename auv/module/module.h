#ifndef __MODULE_H
#define __MODULE_H__

#include <string.h>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/iostreams/concepts.hpp>

#ifdef USE_FTDIPP
    #include <ftdi.hpp>
    #define PICK_FTDI(A,B) A
#else
    #include <ftdi.h>
    #define PICK_FTDI(A,B) B
#endif


#include <common/cauv_utils.h>
#include <common/blocking_queue.h>
#include <generated/messages.h>

class FTDIException : public std::exception
{
    protected:
        int m_errCode;
        std::string m_message;
    public:
        FTDIException(const std::string& msg);
        FTDIException(const std::string& msg, int errCode, PICK_FTDI(Ftdi::Context, ftdi_context)* ftdic);
        
        ~FTDIException() throw();
        virtual const char* what() const throw();
        int errCode() const;
};

typedef boost::shared_ptr<struct usb_device> usb_device_ptr;

class FTDIContext : boost::noncopyable
{
    public:
        void open(int vendor, int product);
        void open(int vendor, int product, int index);
        void open(int vendor, int product, const std::string& description, const std::string& serial = std::string(), int index = 0);
        void setBaudRate(int baudrate);
        void setLineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity);
        void setFlowControl(int flowControl);
        
        std::streamsize read(unsigned char* s, std::streamsize n);
        std::streamsize write(const unsigned char* s, std::streamsize n);

    protected:
        PICK_FTDI(Ftdi::Context, ftdi_context) ftdic;
};


class FTDIDevice : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        FTDIDevice(int vendor, int product, int index,
                   int baudrate,
                   ftdi_bits_type const& bits,
                   ftdi_stopbits_type const& stopBits,
                   ftdi_parity_type const& parity,
                   int flowControl);
        
        std::streamsize read(char* s, std::streamsize n);
        std::streamsize write(const char* s, std::streamsize n);
        
    protected:
        boost::shared_ptr<FTDIContext> m_ftdic;
        boost::shared_ptr<struct usb_device> m_device;

    private:
        const int baudrate;
        const ftdi_bits_type bits;
        const ftdi_stopbits_type stopBits;
        const ftdi_parity_type parity;
        const int flowControl;
};


class Module : public MessageSource
{
    public:
        Module(boost::shared_ptr< std::basic_streambuf<char> > streamBuffer);
        virtual ~Module();
        
        virtual void start();
        void send(boost::shared_ptr<const Message> message);

    protected:
        boost::shared_ptr< std::basic_streambuf<char> > m_streamBuffer;
        boost::thread m_readThread, m_sendThread;
        BlockingQueue< boost::shared_ptr<const Message> > m_sendQueue;

        void sendLoop();
        void readLoop();
};

class FTDIModule : public Module
{
    public:
        FTDIModule(int vendor, int product, int index,
                   int baudrate,
                   ftdi_bits_type const& bits,
                   ftdi_stopbits_type const& stopBits,
                   ftdi_parity_type const& parity,
                   int flowControl);
        
        virtual void start();

    private:
        const int baudrate;
        const ftdi_bits_type bits;
        const ftdi_stopbits_type stopBits;
        const ftdi_parity_type parity;
        const int flowControl;
};

class MCBModule : public FTDIModule
{
    public:
        MCBModule(int index)
            : FTDIModule(0x0403, 0x6001, index, 38400, BITS_8, STOP_BIT_1, NONE, SIO_DISABLE_FLOW_CTRL){
        }
};

#endif // __MODULE_H__
