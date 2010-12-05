#ifndef __PIPELINE_TYPES_H__
#define __PIPELINE_TYPES_H__

#include <string>
#include <exception>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/variant.hpp>

class Node;
class InputNode;
class AsynchronousNode;
class Scheduler;
class ImageProcessor;

class img_pipeline_error: public std::runtime_error{
    public:
        img_pipeline_error(std::string const& str)
            : std::runtime_error(str){
        }
};

class id_error: public img_pipeline_error{
    public:
        id_error(std::string const& str)
            : img_pipeline_error("id error: " + str){
        }
};

class link_error: public img_pipeline_error{
    public:
        link_error(std::string const& str)
            : img_pipeline_error("link error: " + str){
        }
};

class node_type_error: public img_pipeline_error{
    public:
        node_type_error(std::string const& str)
            : img_pipeline_error("node type error: " + str){
        }
};

class scheduler_error: public img_pipeline_error{
    public:
        scheduler_error(std::string const& str)
            : img_pipeline_error("scheduler error: " + str){
        }
};

class parameter_error: public img_pipeline_error{
    public:
        parameter_error(std::string const& str)
            : img_pipeline_error("parameter error: " + str){
        }
};

enum SchedulerPriority {
    priority_slow,
    priority_fast,
    priority_realtime
};

typedef boost::variant<int32_t, float, std::string, bool> param_value_t;

typedef boost::shared_ptr<Node> node_ptr_t;
typedef boost::weak_ptr<Node> node_wkptr_t;

typedef int32_t node_id;
typedef std::string param_id;
typedef std::string input_id;
typedef std::string output_id;

#endif // ndef __PIPELINE_TYPES_H__
