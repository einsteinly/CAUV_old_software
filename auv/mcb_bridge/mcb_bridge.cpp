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

static const std::string delimiter("\xc0\x1d\xbe\xef");

class SerialPort {
    public:
    SerialPort(std::string file, unsigned int baudrate = 112500);
    void read_avail();

    int fd;
    std::vector<int> write_fds;
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
    static const unsigned int header_len = sizeof(data_len) + sizeof(serial_id);
};

SerialPort::SerialPort(std::string file, unsigned int) :
    state(DELIMITING), buf_pos(0), data_len(0), crc(0) {
    fd = open(file.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    termios term;
    tcgetattr(fd, &term);
    cfsetspeed(&term, B115200);
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;
    term.c_lflag &= ~ECHO;
    term.c_iflag &= ~(IXON | IXOFF);
    tcsetattr(fd, TCSANOW, &term);
}

void SerialPort::next_buffer_len(uint16_t len) {
    buf_pos = 0;
    if (buffer.size() < len) {
        buffer = std::string(len, '\0');
    }
    buf_fill_len = len;
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
            }
        }
        next_buffer_len(header_len);
        if (data_len > delimiter.size()) {
            std::cout << "synchronized in " << data_len << " steps" << std::endl;
        }
        //std::cout << "delimited" << std::endl;
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
        //std::cout << "len: " << data_len << std::endl;
        next_buffer_len(data_len);
        state = DATA;
        return;
    }
    if (state == DATA) {
        if (serial_id < write_fds.size()) {
            int ret = write(write_fds[serial_id], &buffer[0], buf_pos);
            if (ret != buf_pos) {
                std::cout << "incomplete write" << std::endl;
            }
        }
        boost::crc_32_type goddammit_boost;
        goddammit_boost.process_bytes(&buffer[0], buf_pos);
        crc = goddammit_boost.checksum() & 0xffff;
        //std::cout << buf_pos << std::endl;
        int val = -1, last_val = -1;
        for (unsigned int ii = 0; ii < buf_pos; ii++) {
            /*val = (int)*(uint8_t*)&buffer[ii];
            if (last_val != -1 && val != last_val + 1) {
                std::cout << "***** ";
            }
            last_val = val;
            std::cout << val << " ";*/
            //std::cout << (int)buffer[ii] << " ";
        }
        std::cout << std::endl;
        next_buffer_len(sizeof(uint16_t));
        state = CRC;
        return;
    }
    if (state == CRC) {
        uint16_t recvd_crc = *(uint16_t*)&buffer[0];
        if (recvd_crc != crc) {
            std::cout << "got: " << recvd_crc << " expected: " << crc << std::endl; 
            std::cout << "len: " << data_len << std::endl; 
            std::cout << "id: " << (int)serial_id << std::endl;
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
    void read_avail();
    int fd;
    int write_fd;
    char ser_id;
    private:
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
    term.c_lflag &= ~ECHO;
    tcsetattr(slave, TCSANOW, &term);
}

void Pty::read_avail() {
    int ret = read(fd, &buffer[0], buffer.size());
    if (ret <= 0) {
        return;
    }
    uint16_t len = ret; 
    boost::crc_32_type goddammit_boost;
    goddammit_boost.process_bytes(&buffer[0], len);
    uint16_t crc = goddammit_boost.checksum() & 0xffff;
    if (write_fd > 0) {
        write(write_fd, &delimiter[0], delimiter.size());
        write(write_fd, &ser_id, 1);
        write(write_fd, (char*)&len, sizeof(len));
        write(write_fd, &buffer[0], len);
        write(write_fd, (char*)&crc, sizeof(crc));
    }
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    po::options_description desc("bridge options");

    std::string port_name;
    std::string port_prefix;
    unsigned int n_ports;
    desc.add_options()
        ("help,h", "Print this help message")
        ("port,p", po::value<std::string>(&port_name)->default_value("/dev/ttyUSB0"), "Serial port to connect to")
        ("prefix,x", po::value<std::string>(&port_prefix)->default_value("/dev/ttyV"), "Prefix for virtual serial ports")
        ("n_ports,n", po::value<unsigned int>(&n_ports)->default_value(3), "Number of ports to multiplex");

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
    }
}
