#ifndef __PIPELINE_TYPES_H__
#define __PIPELINE_TYPES_H__

#include <string>
#include <exception>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

class Node;
enum Priority {
    low,
    high,
    realtime
};

// until such time as one exists...
class Scheduler{
    public:
        void addToQueue(Node* n, Priority p){}
};


class img_pipeline_error: public std::runtime_error{
    public:
        img_pipeline_error(std::string const& str)
            : std::runtime_error(str){
        }
};

class id_error: public img_pipeline_error{
    public:
        id_error(std::string const& str)
            : img_pipeline_error(str){
        }
};

class link_error: public img_pipeline_error{
    public:
        link_error(std::string const& str)
            : img_pipeline_error(str){
        }
};

class node_type_error: public img_pipeline_error{
    public:
        node_type_error(std::string const& str)
            : img_pipeline_error(str){
        }
};


typedef boost::variant<float, int, std::string> param_value_t;

typedef boost::shared_ptr<Node> node_ptr_t;

typedef std::string node_id;
typedef std::string param_id;
typedef std::string input_id;
typedef std::string output_id;

#endif // ndef __PIPELINE_TYPES_H__
