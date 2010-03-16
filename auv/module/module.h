#ifndef __MODULE_H
#define __MODULE_H__

#include <ftdi.h>
#include <iostream>
#include <string.h>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/thread.hpp>

#include <common/cauv_utils.h>
#include <common/cauv_global.h>
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

template<int baudrate, ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity, int flowControl>
class FTDIStream : public boost::iostreams::device<boost::iostreams::bidirectional>
{
    public:
        FTDIStream(int deviceID) throw(FTDIException)
        {
            ftdi_init(&ftdic);

            struct ftdi_device_list* devices,* curdevices;
            int ret;

            if ((ret = ftdi_usb_find_all(&ftdic, &devices, 0x0403, 0x6001)) < 0)
            {
                throw FTDIException("ftdi_usb_find_all failed", ret, &ftdic);
            }

            int deviceCount = ret;

            cauv_global::trace(MakeString() << "Number of devices found: " << deviceCount);

            if (deviceCount == 0)
                throw FTDIException("Could not open device (No devices found)");

            curdevices = devices;
            int i = deviceID;
            while (i--)
            {
                if (curdevices == 0)
                {
                    throw FTDIException("Could not open device (No matching device found)");
                }
                curdevices = curdevices->next;
            }

            boost::scoped_ptr<struct usb_device> device(curdevices->dev);

            ftdi_list_free(&devices);

            cauv_global::trace(MakeString() << "Module's device (id:" << deviceID << ") found...");

            if ((ret = ftdi_usb_open_dev(&ftdic, device.get())) < 0)
            {
                throw FTDIException("Unable to open ftdi device", ret, &ftdic);
            }
            
            // Set properties
            if ((ret = ftdi_set_baudrate(&ftdic, baudrate)) < 0)
            {
                throw FTDIException("Unable to set baudrate", ret, &ftdic);
            }
            if ((ret = ftdi_set_line_property(&ftdic, bits, stopBits, parity)) < 0)
            {
                throw FTDIException("Unable to set line property", ret, &ftdic);
            }
            if ((ret = ftdi_setflowctrl(&ftdic, flowControl)) < 0)
            {
                throw FTDIException("Unable to set flow control", ret, &ftdic);
            }

            ftdi_initialised = true;
        }
        
        
        std::streamsize read(char* s, std::streamsize n)
        {
            int ret = ftdi_read_data(&ftdic, reinterpret_cast<u_char*>(s), n);
            if (ret < 0)
            {
                throw FTDIException("Unable to read from ftdi device", ret, &ftdic);
            }
            return ret;
        }

        std::streamsize write(const char* s, std::streamsize n)
        {
            int ret = ftdi_write_data(&ftdic, const_cast<u_char*>(reinterpret_cast<const u_char*>(s)), n);
            if (ret < 0)
            {
                throw FTDIException("unable to write to ftdi device", ret, &ftdic);
            }
            return ret;
        }
        
        void close() 
        {
            if (ftdi_initialised)
            {
                ftdi_usb_close(&ftdic);
                ftdi_deinit(&ftdic);
                ftdi_initialised = false;
            }
        }

        void close(std::ios_base::openmode which) 
        {
            close();
        } 

    protected:
        struct ftdi_context ftdic;
        bool ftdi_initialised;
};


template<int baudrate, ftdi_bits_type bits, ftdi_stopbits_type stopBits, ftdi_parity_type parity, int flowControl>
class Module : public MessageSource
{
    typedef FTDIStream<baudrate, bits, stopBits, parity, flowControl> ftdi_stream_t;
    
    public:
        Module(int deviceID) throw(FTDIException) : ftdiStream(deviceID)
        {
            cauv_global::trace("USB opened...");

            m_running = true;
            m_readThread = boost::make_shared<boost::thread>(&Module::readLoop, this);

            cauv_global::trace("Module thread started");
        }

        ~Module()
        {
            m_running = false;
        }


        void send(Message& message) throw (FTDIException)
        {
            boost::iostreams::stream<ftdi_stream_t> ftdi(ftdiStream);

            byte_vec_t bytes = message.toBytes();


            uint32_t buf[2];
            buf[0] = 0xdeadc01d;
            buf[1] = bytes.size();
            uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf); 
            std::vector<uint16_t> vheader(buf16, buf16+4);
            uint16_t checksum = sumOnesComplement(vheader);


            ftdi << 0xdeadc01d;
            ftdi << bytes.size();
            ftdi << checksum;
            ftdi << bytes;
        }

    protected:
        ftdi_stream_t ftdiStream;
        boost::shared_ptr<boost::thread> m_readThread;
        volatile bool m_running;

        void readLoop()
        {
            byte_vec_t curMsg;

            boost::iostreams::stream<ftdi_stream_t> ftdi(ftdiStream);

            while (m_running)
            {
                uint32_t startWord;
                ftdi >> startWord;

                if (startWord != 0xdeadc01d)
                {
                    // Start word doesn't match, drop it
                    continue;
                }

                uint32_t len;
                uint16_t checksum;
                ftdi >> len;
                ftdi >> checksum;

                uint32_t buf[2];
                buf[0] = startWord;
                buf[1] = len;

                uint16_t* buf16 = reinterpret_cast<uint16_t*>(buf); 
                std::vector<uint16_t> vheader(buf16, buf16+4);

                if (sumOnesComplement(vheader) != checksum)
                {
                    // Checksum doesn't match, drop it
                    continue;
                }

                curMsg.clear();
                char c;
                for (uint32_t i = 0; i < len; ++i)
                {
                    ftdi >> c;
                    curMsg.push_back(c);
                }

                std::cout << "Message from module  [ len: " << len << " | checksum: " << checksum << " ]" << std::endl;
                foreach(char c, curMsg)
                {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << c << " " << std::dec;
                }
                std::cout << std::endl;

                this->notifyObservers(curMsg);
            }
        }
};


typedef Module<38400, BITS_8, STOP_BIT_1, NONE, SIO_DISABLE_FLOW_CTRL> MCBModule;

#endif // __MODULE_H__
