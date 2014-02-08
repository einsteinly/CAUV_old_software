#pragma once
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>

namespace cauv {

template <class Message>
void load_message(std::string filename, Message &message) {
    namespace fs = boost::filesystem;
    fs::path file(filename.c_str());
    int file_len = fs::file_size(file);
    std::ifstream fstream(file.c_str(), std::ifstream::binary);

    std::vector<uint8_t> msg_data;
    msg_data.reserve(file_len);
    fstream.read((char*)&msg_data[0], file_len);

    ros::serialization::IStream istream(&msg_data[0], file_len);
    ros::serialization::deserialize(istream, message);
}

template <class Message>
void save_message(std::string filename, Message &message) {
    std::ofstream out_file;
    out_file.open(filename.c_str(), std::ofstream::binary);
    auto serialized_msg = ros::serialization::serializeMessage(message);
    out_file.write(reinterpret_cast<const char*>(serialized_msg.message_start), serialized_msg.num_bytes);
    out_file.close();
}

}
