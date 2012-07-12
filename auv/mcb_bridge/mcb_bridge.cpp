#include <iostream>
#include <stdexcept>
#include <vector>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdint.h>
#include <poll.h>
#include <pty.h>

#include <boost/program_options.hpp>
#include <boost/crc.hpp>
#include <boost/lexical_cast.hpp>
#include <debug/cauv_debug.h>
#include <utility/files.h>

//imported from embedded_software. Needs to be synced
#define CXX_HACKY_HACK
#include "frames.h"
#undef CXX_HACKY_HACK

static const std::string delimiter("\xc0\x1d\xbe\xef");

#define CAN_SERIAL_ID 255

class SerialPort {
    public:
    SerialPort(std::string file, unsigned int baudrate = 115200);
    void read_avail();

    int fd;
    std::vector<int> write_fds;
    std::vector<int> can_fds;
    private:
    enum {
        DELIMITING,
        HEADER,
        DATA,
        CRC,
    } state;
    std::string buffer;
    uint16_t buf_pos;
    uint16_t data_len;
    uint16_t buf_fill_len;
    uint8_t serial_id;
    uint16_t crc;
    void next_buffer_len(uint16_t len);
    static const unsigned int header_len = sizeof(uint16_t) + sizeof(uint8_t);//sizeof(data_len) + sizeof(serial_id);
};

SerialPort::SerialPort(std::string file, unsigned int) :
    state(DELIMITING), buf_pos(0), data_len(0), crc(0) {
    fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    termios term;
    tcgetattr(fd, &term);
    cfsetspeed(&term, B115200);

    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;
    term.c_cflag &= ~CRTSCTS;
    term.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    term.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

    term.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                      ONOCR | OFILL | OPOST);

    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;

    term.c_cflag |= (CLOCAL | CREAD);

    tcflush(fd, TCIOFLUSH); 
    tcsetattr(fd, TCSANOW, &term);
}

void SerialPort::next_buffer_len(uint16_t len) {
    buf_pos = 0;
    if (buffer.size() < len) {
        buffer = std::string(len, '\0');
    }
    buf_fill_len = len;
}

template <class T>
T msg_from_frame(can_frame_t &frame) {
    T msg;
    memcpy(&msg.frame, &frame, sizeof(frame));
}

void SerialPort::read_avail() {
    int ret = 0;
    if (state == DELIMITING) {
        char c;
        while(buf_pos < delimiter.size()) {
            ret = read(fd, &c, sizeof(c));
            if (ret <= 0) {
                return;
            }
            data_len++;
            if (c == delimiter[buf_pos]) {
                buf_pos++;
            } else {
                buf_pos = 0;
            }
        }
        next_buffer_len(header_len);
        if (data_len > delimiter.size()) {
            debug() << "synchronized in " << data_len << " steps";
        }
        debug(7) << "delimited" << std::endl;
        state = HEADER;
    }
    ret = read(fd, &buffer[buf_pos], buf_fill_len - buf_pos);
    if (ret >= 0) {
        buf_pos += ret;
    }
    if (buf_pos < buf_fill_len) {
        return;
    }
    if (state == HEADER) {
        serial_id = buffer[0];
        data_len = *(uint16_t*)&buffer[1];
        next_buffer_len(data_len);
        state = DATA;
        return;
    }
    if (state == DATA) {
        if (serial_id == CAN_SERIAL_ID) {
            if (buf_pos > sizeof(can_frame_t)) {
                warning() << "Wrong size buffer (" << buf_pos << ") for can frame!" << std::endl;
            } else {
                can_frame_t frame;
                memset(&frame, 0, sizeof(frame));
                memcpy(&frame, &buffer[0], buf_pos);
                debug(2) << "CAN frame: id: " << frame.id << " len: " << (int)frame.len << std::endl;
                for (unsigned int ii = 0; ii < frame.len; ii++) {
                    debug(5) << (unsigned int)frame.data[ii] << " ";
                }
                for (unsigned int ii = 0; ii < can_fds.size(); ii++) {
                    write(can_fds[ii], &delimiter[0], delimiter.size());
                    write(can_fds[ii], (uint8_t*)&frame, sizeof(frame));
                }
            }
        } else if (serial_id < write_fds.size()) {
            int ret = write(write_fds[serial_id], &buffer[0], buf_pos);
            if (ret != buf_pos) {
                debug(3) << "incomplete write" << std::endl;
            }
        } else {
            warning() << "Unknown serial id: " << serial_id << std::endl;
        }
        boost::crc_32_type goddammit_boost;
        goddammit_boost.process_bytes(&buffer[0], buf_pos);
        crc = goddammit_boost.checksum() & 0xffff;
        //std::cout << buf_pos << std::endl;
        //int val = -1, last_val = -1;
        for (unsigned int ii = 0; ii < buf_pos; ii++) {
            /*val = (int)*(uint8_t*)&buffer[ii];
            if (last_val != -1 && val != last_val + 1) {
                std::cout << "***** ";
            }
            last_val = val;
            std::cout << val << " ";*/
            //std::cout << (int)buffer[ii] << " ";
        }
        //std::cout << std::endl;
        next_buffer_len(sizeof(uint16_t));
        state = CRC;
        return;
    }
    if (state == CRC) {
        uint16_t recvd_crc = *(uint16_t*)&buffer[0];
        if (recvd_crc != crc) {
            error() << "got: " << recvd_crc << " expected: " << crc << std::endl; 
            error() << "len: " << data_len << std::endl; 
            error() << "id: " << (int)serial_id << std::endl;
        }
        state = DELIMITING;
        data_len = 0;
        buf_pos = 0;
        return;
    }
};

class Pty {
    public:
    Pty(std::string name);
    virtual void read_avail();
    int fd;
    int write_fd;
    uint8_t ser_id;
    protected:
    void write_buf(const char* buf, uint16_t len);
    std::string buffer;
}; 

Pty::Pty(std::string name) :
    write_fd(-1) ,
    buffer(400, '\0') {
    int master, slave;
    openpty(&master, &slave, NULL, NULL, NULL);
    std::string pty_name(ttyname(slave));
    std::cout << pty_name << "->" << name << std::endl;
    unlink(name.c_str());
    symlink(pty_name.c_str(), name.c_str());
    fd = master;
    termios term;
    tcgetattr(slave, &term);
    cfsetspeed(&term, B115200);
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;
    term.c_cflag &= ~CRTSCTS;
    term.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    term.c_iflag &=  ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                     | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | INPCK );

    term.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                      ONOCR | OFILL | OPOST);

    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;

    term.c_cflag |= (CLOCAL | CREAD);

    tcflush(slave, TCIOFLUSH); 
    tcflush(master, TCIOFLUSH); 
    tcsetattr(slave, TCSANOW, &term);
    tcsetattr(master, TCSANOW, &term);
    fcntl(fd, F_SETFL, FNDELAY);
}

void Pty::read_avail() {
    int ret = read(fd, &buffer[0], buffer.size());
    if (ret <= 0) {
        return;
    }
    write_buf(&buffer[0], ret);
}

void Pty::write_buf(const char *buf, uint16_t len) {
    boost::crc_32_type goddammit_boost;
    goddammit_boost.process_bytes(&buffer[0], len);
    uint16_t crc = goddammit_boost.checksum() & 0xffff;
    if (write_fd > 0) {
        write(write_fd, &delimiter[0], delimiter.size());
        write(write_fd, &ser_id, 1);
        write(write_fd, (char*)&len, sizeof(len));
        write(write_fd, buf, len);
        write(write_fd, (char*)&crc, sizeof(crc));
    }
}

class CANPty : public Pty { 
    public:
    CANPty(std::string name);
    virtual void read_avail();
    private:
    enum {
        DELIMITING,
        READING
    } state;
    unsigned int buf_pos;
};

CANPty::CANPty (std::string name) :
    Pty(name), state(DELIMITING), buf_pos(0) {
} 

void CANPty::read_avail() {
    if (state == DELIMITING) {
        while(buf_pos < delimiter.size()) {
            char c;
            int ret = read(fd, &c, sizeof(c));
            if (ret <= 0) {
                return;
            }
            if (c == delimiter[buf_pos]) {
                buf_pos++;
            } else {
                buf_pos = 0;
            }
        }
        buf_pos = 0;
        state = READING;
        debug() << "CAN Delimited";
    } else if (state == READING) {
        can_frame_t frame;
        int ret = read(fd, ((char*)&frame) + buf_pos, sizeof(frame) - buf_pos); 
        if (ret >= 0) {
            buf_pos += ret;
        } 
        if (buf_pos < sizeof(frame)) {
            return;
        }
        debug() << "Write CAN frame id: " << frame.id;
        write_buf((char*)&frame, sizeof(frame));
        state = DELIMITING;
        buf_pos = 0;
    }
}

int main(int argc, char **argv) {
    if (!get_lock_file("/tmp/mcb_bridge")) {
        error() << "Cannot get lock file. Another bridge is still running!";
        return 1;
    }
    namespace po = boost::program_options;
    po::options_description desc("bridge options");

    std::string port_name;
    std::string port_prefix;
    unsigned int n_ports;
    unsigned int n_can_ports;
    desc.add_options()
        ("help,h", "Print this help message")
        ("port,p", po::value<std::string>(&port_name)->default_value("/dev/ttyUSB0"), "Serial port to connect to")
        ("prefix,x", po::value<std::string>(&port_prefix)->default_value("/dev/ttyV"), "Prefix for virtual serial ports")
        ("n_ports,n", po::value<unsigned int>(&n_ports)->default_value(3), "Number of ports to multiplex")
        ("n_can_ports,c", po::value<unsigned int>(&n_can_ports)->default_value(2), "Number of CAN ports to produce");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);
    if(vm.count("help")) {
        std::cout << desc << std::flush;
        return 1;
    }
    
    SerialPort serial(port_name);
    std::vector<Pty> ptys;
    std::vector<pollfd> fd_vect(1);
    fd_vect[0].fd = serial.fd;
    fd_vect[0].events = POLLIN;
    for(unsigned int ii = 0; ii < n_ports; ii++) {
        Pty pty(port_prefix + boost::lexical_cast<std::string>(ii));
        serial.write_fds.push_back(pty.fd);
        pty.write_fd = serial.fd;
        pty.ser_id = ii;
        pollfd p;
        p.fd = pty.fd;
        p.events = POLLIN;
        ptys.push_back(pty);
        fd_vect.push_back(p);
    }
    std::vector<CANPty> can_ptys;
    for (unsigned int ii = 0; ii < n_ports; ii++) {
        CANPty can(port_prefix + "CAN" + boost::lexical_cast<std::string>(ii));
        serial.can_fds.push_back(can.fd);
        can.write_fd = serial.fd;
        can.ser_id = 255;
        pollfd p;
        p.fd = can.fd;
        p.events = POLLIN;
        ptys.push_back(can);
        fd_vect.push_back(p);
    }
    while(true) {
        int ret = poll(fd_vect.data(), fd_vect.size(), 1000); 
        if (fd_vect[0].revents & POLLIN) {
            serial.read_avail();
        }
        for (unsigned int ii = 0; ii < n_ports; ii++) {
            if (fd_vect[ii + 1].revents & POLLIN) {
                ptys[ii].read_avail();
            }
        }
        for (unsigned int ii = 0; ii < n_can_ports; ii++) {
            if (fd_vect[n_ports + ii + 1].revents & POLLIN) {
                can_ptys[ii].read_avail();
            }
        }
    }
}
