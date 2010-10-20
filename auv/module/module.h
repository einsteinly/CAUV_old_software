#ifndef __MODULE_H
#define __MODULE_H__

#include <ftdi.h>
#include <string.h>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/concepts.hpp>

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
        FTDIException(const std::string& msg, int errCode, ftdi_context* ftdic);
        ~FTDIException() throw();
        virtual const char* what() const throw();
        int errCode() const;
};

typedef boost::shared_ptr<struct usb_device> usb_device_ptr;

class FTDIContext : boost::noncopyable
{
    public:
        FTDIContext();
        ~FTDIContext();

        std::vector<usb_device_ptr> usbFindAll();

        void openDevice(usb_device_ptr dev);
        void setBaudRate(int baudrate);
        void setLineProperty(ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity);
        void setFlowControl(int flowControl);
        
        std::streamsize read(unsigned char* s, std::streamsize n);
        std::streamsize write(const unsigned char* s, std::streamsize n);

    protected:
        struct ftdi_context ftdic;
        bool m_usb_open;
};


class FTDIDevice : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        FTDIDevice(int baudrate,
                   ftdi_bits_type const& bits,
                   ftdi_stopbits_type const& stopBits,
                   ftdi_parity_type const& parity,
                   int flowControl,
                   int deviceID);
        
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
        Module(int baudrate,
               ftdi_bits_type const& bits,
               ftdi_stopbits_type const& stopBits,
               ftdi_parity_type const& parity,
               int flowControl,
               int deviceID);

        void start();
        ~Module();
        void send(boost::shared_ptr<const Message> message);

    protected:
        boost::iostreams::stream_buffer<FTDIDevice> m_ftdiStreamBuffer;
        boost::thread m_readThread, m_sendThread;
        BlockingQueue< boost::shared_ptr<const Message> > m_sendQueue;

        void sendLoop();
        void readLoop();

    private:
        const int baudrate;
        const ftdi_bits_type bits;
        const ftdi_stopbits_type stopBits;
        const ftdi_parity_type parity;
        const int flowControl;
};

struct MCBModule: Module
{
    MCBModule(int deviceID)
        : Module(38400, BITS_8, STOP_BIT_1, NONE, SIO_DISABLE_FLOW_CTRL, deviceID){
    }
};

#endif // __MODULE_H__
