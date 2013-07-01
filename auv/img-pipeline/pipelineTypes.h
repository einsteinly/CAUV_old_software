/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __PIPELINE_TYPES_H__
#define __PIPELINE_TYPES_H__

#include <string>
#include <exception>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/variant.hpp>

namespace cauv{
namespace imgproc{

class Node;
class InputNode;
class AsynchronousNode;
class Scheduler;
class ImageProcessor;

enum SchedulerPriority {
    priority_slow,
    priority_fast,
    priority_fastest
};

typedef boost::shared_ptr<Node> node_ptr_t;
typedef boost::weak_ptr<Node> node_wkptr_t;

typedef int32_t node_id;
typedef std::string input_id;
typedef std::string output_id;

class img_pipeline_error: public std::runtime_error{
    public:
        img_pipeline_error(const std::string& str)
            : std::runtime_error(str){
        }
};

class id_error: public img_pipeline_error{
    public:
        id_error(const std::string& str)
            : img_pipeline_error("id error: " + str){
        }
};

class link_error: public img_pipeline_error{
    public:
        link_error(const std::string& str)
            : img_pipeline_error("link error: " + str){
        }
};

class node_type_error: public img_pipeline_error{
    public:
        node_type_error(const std::string& str)
            : img_pipeline_error("node type error: " + str){
        }
};

class scheduler_error: public img_pipeline_error{
    public:
        scheduler_error(const std::string& str)
            : img_pipeline_error("scheduler error: " + str){
        }
};

class parameter_error: public img_pipeline_error{
    public:
        parameter_error(const std::string& str)
            : img_pipeline_error("parameter error: " + str){
        }
};

class bad_input_error: public img_pipeline_error{
    public:
        bad_input_error(input_id const& iid, const std::string& str = "")
            : img_pipeline_error("parameter error: " + str), m_iid(iid){
        }
        virtual ~bad_input_error() throw(){
        }
        input_id const& inputId() const{
            return m_iid;
        }
    private:
        input_id m_iid;
};

class user_attention_error: public img_pipeline_error{
    public:
        user_attention_error(const std::string& str)
            : img_pipeline_error("parameter error: " + str){
        }
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PIPELINE_TYPES_H__
