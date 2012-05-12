/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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

class bad_input_error: public img_pipeline_error{
    public:
        bad_input_error(input_id const& iid, std::string const& str = "")
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
        user_attention_error(std::string const& str)
            : img_pipeline_error("parameter error: " + str){
        }
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PIPELINE_TYPES_H__
