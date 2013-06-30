/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __NODE_H__
#define __NODE_H__

#include <map>
#include <list>
#include <set>
#include <stdexcept>
#include <iostream>
#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/variant.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <utility/string.h>
#include <utility/testable.h>
#include <utility/throughput.h>
#include <utility/ratelimit.h>
#include <common/msg_classes/image.h>
#include <common/mailbox.h>
#include <debug/cauv_debug.h>
#include <generated/types/PipelineGroup.h>
#include <generated/types/Pl_GuiGroup.h>
#include <generated/types/NodeInputStatus.h>
#include <generated/types/InputSchedType.h>
#include <generated/types/SensorUIDBase.h>
#include <generated/types/UID.h>
#include <generated/types/GPSLocationMessage.h>
#include <generated/types/TelemetryMessage.h>

#include "pipelineTypes.h"

namespace cauv{
namespace imgproc{

/* Getter for ParamValue. Specialised for the case where we want the
 * actual variant. This is also the mechanism by which compatible types are
 * converted.
 * The compatible type conversions here should correspond to the
 * CompatibleSubTypes visitor defined below.
 */
template<typename T>
inline T getValue(const ParamValue& v) {
    return boost::get<T>(v);
}
template<>
inline float getValue<float>(const ParamValue& v) {
    try{
        return boost::get<float>(v);
    }catch(boost::bad_get&){
        return boost::get<BoundedFloat>(v);
    }
}
template<>
inline int32_t getValue<int32_t>(const ParamValue& v) {
    try{
        return boost::get<int32_t>(v);
    }catch(boost::bad_get&){
        return int32_t(float(boost::get<BoundedFloat>(v)));
    }
}
template<>
inline BoundedFloat getValue<BoundedFloat>(const ParamValue& v) {
    try{
        return boost::get<BoundedFloat>(v);
    }catch(boost::bad_get&){}

    // in these cases no range information is available!
    try{
        return BoundedFloat(boost::get<float>(v), 0, 0, BoundedFloatType::Clamps);
    }catch(boost::bad_get&){}

    return BoundedFloat(boost::get<int32_t>(v), 0, 0, BoundedFloatType::Clamps);
}
template<>
inline ParamValue getValue<ParamValue>(const ParamValue& v) {
    return v;
}


struct CompatibleSubTypes: public boost::static_visitor< std::vector<int32_t> >{
    template<typename T>
    std::vector<int32_t> operator()(T const& v) const{
        std::vector<int32_t> r;
        r.push_back(ParamValue(v).which());
        return r;
    }
    std::vector<int32_t> operator()(float const&) const{
        std::vector<int32_t> r;
        r.push_back(ParamValue(float()).which());
        r.push_back(ParamValue(BoundedFloat()).which());
        return r;
    }
    std::vector<int32_t> operator()(int32_t const&) const{
        std::vector<int32_t> r;
        r.push_back(ParamValue(int32_t()).which());
        r.push_back(ParamValue(BoundedFloat()).which());
        return r;
    }
    std::vector<int32_t> operator()(BoundedFloat const&) const{
        std::vector<int32_t> r;
        r.push_back(ParamValue(BoundedFloat()).which());
        r.push_back(ParamValue(float()).which());
        r.push_back(ParamValue(int32_t()).which());
        return r;
    }
};

// !!! TODO: remove these and scope as appropriate
using InputSchedType::Must_Be_New;
using InputSchedType::May_Be_Old;
using InputSchedType::Optional;

class Node: public boost::enable_shared_from_this<Node>, boost::noncopyable{
    public:
        // Public typedefs: used as return types
        typedef std::vector<NodeInput> msg_node_in_list_t;
        typedef std::map<LocalNodeOutput, msg_node_in_list_t> msg_node_output_map_t;
        typedef std::map<LocalNodeInput, NodeOutput> msg_node_input_map_t;
        typedef std::map<LocalNodeInput, ParamValue> msg_node_param_map_t;

        typedef std::set<output_id> output_id_set_t;
        typedef std::set<input_id> input_id_set_t;

    protected:
        // Protected types and typedefs: useful for derived nodes

        // Actually this one should not normally be of interest to derived
        // types, except perhaps for debug purposes...
        // Inside the image pipeline we track the origins of data with
        // UIDs tagged onto every parameter.
        // Image objects also have an internal UID - the UID of a parameter is
        // set to that of the image that it was derived from if the value was
        // derived from an image.
        struct InternalParamValue{
            typedef InternalParamValue this_t;

            ParamValue param;
            UID        uid;

            InternalParamValue();
            InternalParamValue(ParamValue const& param, UID const& uid);
            explicit InternalParamValue(ParamValue const& param);

            template<typename T>
            InternalParamValue(T const& v, UID const& uid)
                : param(v), uid(uid){
            }

            operator ParamValue() const;
            bool operator==(InternalParamValue const& other) const;
        };
        template<typename cT, typename tT>
        friend std::basic_ostream<cT, tT>& operator<<(
            std::basic_ostream<cT, tT>&, InternalParamValue const&
        );

        typedef boost::shared_ptr<Image> image_ptr_t;

        // NB: order here is important, don't change it!
        // matches cauv::OutputType::e
        typedef boost::variant<image_ptr_t, InternalParamValue> output_t;
        
        // visitor to get UID from output_t
        struct output_t_GetUID: boost::static_visitor<UID>{
            UID operator()(image_ptr_t const& img) const{
                if(img)
                    return img->id();
                return UID();
            }
            UID operator()(InternalParamValue const& ipv) const{
                return ipv.uid;
            }
        };

        typedef std::map<input_id, image_ptr_t> in_image_map_t;

        class _OutMap: public std::map<output_id, output_t>{
            private:
                typedef std::map<output_id, output_t> base_t;
            public:
                // constructed from the inputs that the outputs will be derived
                // from - the UIDs of the inputs are used to decide the UID
                // used for outputs
                _OutMap(in_image_map_t const& inputs);
                // yay, triply nested class:
                struct UIDAddingProxy{
                    UIDAddingProxy(output_t& ref, UID const& uid)
                        : m_ref(ref), m_uid_to_add(uid){
                    }
                    // return void to stop people doing crazy operator chaining
                    // and confusing themselves:
                    inline void operator=(ParamValue const& v){
                        m_ref = output_t(InternalParamValue(v, m_uid_to_add));
                    }
                    // if an image is assigned, same rule applies - but images
                    // have an internal UID instead of being wrapped in a
                    // pipeline-specific structure:
                    // Images can also be NULL...
                    inline void operator=(image_ptr_t const& img){
                        if(img)
                            img->id(m_uid_to_add);
                        m_ref = output_t(img);
                    }

                    output_t& m_ref;
                    UID const& m_uid_to_add;
                };
                // use this operator to set outputs in the map: it
                // automatically attaches a UID as the value is added to the
                // internal map
                inline UIDAddingProxy operator[](output_id const& oid){
                    return UIDAddingProxy(base_t::operator[](oid), m_uid);
                }
                // Note that derived node types *can* use the base operator[]
                // to specify internal paramvalues directly; the base operator
                // is provided by this function in order to be explicit: This
                // access method is required if you want to set a
                // non-automagical UID. (i.e. by input nodes);
                inline output_t& internalValue(output_id const& oid){
                    return base_t::operator[](oid);
                }
                UID const& uidThatWillBeAdded() const{
                    return m_uid;
                }
            private:
                UID m_uid;
        };

        typedef _OutMap out_map_t;
        
        // used to tag inputs to aid automatic copying
        enum ConstQualifier{NonConst, Const};        

    private:
        // Private types and typedefs: only used internally

        enum InputType {InType_Parameter, InType_Image};
        template<typename cT, typename tT>
        friend std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>&, InputType const&);

        template<typename T_ID>
        struct link_target: TestableBase< link_target<T_ID> > {
            typedef node_ptr_t node_t;
            typedef T_ID id_t;
            node_t node;
            id_t id;

            link_target()
                : TestableBase< link_target<T_ID> >(*this), node(), id(){
            }

            link_target(node_t n, id_t i)
                : TestableBase< link_target<T_ID> >(*this), node(n), id(i){
            }

            void clearConnection() { node = node_t(); id = id_t(); }

            bool valid() const{
                //debug() << "valid():" << node << id;
                return !!node;
            }

            bool operator==(const link_target<T_ID>& other) const {
                return node == other.node && id == other.id;
            }

            bool operator!=(const link_target<T_ID>& other) const {
                return node != other.node || id != other.id;
            }
        };
        template<typename char_T, typename traits, typename T_ID>
        friend std::basic_ostream<char_T, traits>& operator<<(
            std::basic_ostream<char_T, traits>& os, Node::link_target<T_ID> const& l);


        typedef link_target<output_id> input_link_t;
        struct Input;
        typedef boost::shared_ptr<Input> input_ptr;
        struct Input: TestableBase<Input>{
            input_link_t target;
            InputSchedType::e sched_type;
            NodeInputStatus::e status;
            InputType input_type;
            mutable InternalParamValue param_value;
            std::set<int32_t> compatible_subtypes;
            std::string tip;
            input_ptr synchronised_with;
            ConstQualifier isconst;

            Input(InputSchedType::e s,
                  ConstQualifier isconst);
            Input(InputSchedType::e s,
                  ParamValue const& default_value,
                  const std::string& tip,
                  std::vector<int32_t> const& compatible_subtypes);

            static input_ptr makeImageInputShared(ConstQualifier isconst,
                                                  InputSchedType::e const& st = Must_Be_New);
            static input_ptr makeParamInputShared(ParamValue const& default_value,
                                                  const std::string& tip,
                                                  InputSchedType::e const& st = May_Be_Old);
            void clear();
            image_ptr_t getImage() const;
            // NB: no locks are necessarily held when calling these:
            InternalParamValue getParam(bool& did_change) const;
            InternalParamValue getParam(bool& did_change, UID const&) const;
            bool valid() const;
            bool isParam() const;
            bool constQualifier() const;
            
            UID latestUIDSharedWithSync() const;            
            bool synchronisedInputAvailable() const;
        };
        template<typename cT, typename tT>
        friend std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>&, Input const&);

        typedef link_target<input_id> output_link_t;
        typedef std::list<output_link_t> output_link_list_t;
        struct Output;
        typedef boost::shared_ptr<Output> output_ptr;
        struct Output{
            output_link_list_t targets;
            std::deque<output_t> value_queue;
            int should_queue; // actually a count of connected inputs that require queuing
            bool demanded;

            Output(output_t const& value)
                : targets(), value_queue(1, value), should_queue(0), demanded(false){
            }

            void pushShouldQueue(){ should_queue++; }
            void popShouldQueue(){ should_queue--; assert(should_queue >= 0); }

            output_t const& value() const{
                return value_queue.back();
            } 

            output_t const& popToValueWithUID(UID const& id){
                // (! ==) == (!=)
                while(!(boost::apply_visitor(output_t_GetUID(), value_queue.front()) == id) &&
                      value_queue.size() > 1){
                    value_queue.pop_front();
                }
                if(!(boost::apply_visitor(output_t_GetUID(), value_queue.front()) == id)){
                    error() << __func__ << "not possible!";
                }
                return value_queue.front();
            }

            void setValue(output_t const& value){
                if(should_queue)
                    value_queue.push_back(value);
                else
                    value_queue.back() = value;
            }

            int which() const{
                return value_queue.back().which();
            }

            bool isParam() const{
                return value_queue.back().which() == OutputType::Parameter;
            }
        };
        template<typename cT, typename tT>
        friend std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>&, Output const&);

        typedef std::map<output_id, output_ptr> private_out_map_t;
        typedef std::map<input_id,  input_ptr>  private_in_map_t;

        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;

    public:
        struct ConstructArgs{
            Scheduler& sched; ImageProcessor& pl; const std::string& pl_name; NodeType::e type;
            ConstructArgs(Scheduler& sched, ImageProcessor& pl, const std::string& pl_name, NodeType::e type);
        };
        Node(ConstructArgs const& args);

        /* non-trivial construction MUST be done in init(): non-trivial means
         * anything that relies on calling ANY method of this base class
         */
        virtual void init() = 0;


        virtual ~Node();

        /* Destructors of derived types that have any member variables should
         * call this FIRST.
         * This function should ONLY be called from a destructor.
         */
        void stop();

        NodeType::e const& type() const;
        node_id const& id() const;
        const std::string& plName() const;

        // TODO: rename this to connectInput
        void setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id);
        // TODO: and rename these to disconnect..
        void clearInput(input_id const& i_id);
        void clearInputs(node_ptr_t parent);
        void clearInputs();

        /* parameters DO NOT count */
        input_id_set_t inputs() const;
        /* parameters DO count, return everything not just connected things */
        msg_node_input_map_t inputLinks() const;
        /* parameters DO count */
        std::set<node_ptr_t> parents() const;

        // TODO: rename this to connectOutput
        void setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id);
        // TODO: and rename these to disconnect..
        void clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id);
        void clearOutputs(node_ptr_t child);
        void clearOutputs();

        output_id_set_t outputs(int type_index) const;
        output_id_set_t outputs() const;
        output_id_set_t paramOutputs() const;

        int32_t paramOutputType(output_id const& o_id) const;

        // TODO NPA: include param children
        /* parameters DO count, return everything not just connected things */
        msg_node_output_map_t outputLinks() const;
        const output_link_list_t& linksOnOutput(output_id const& o) const;
        bool hasChildOnOutput(output_id const&) const;

        // TODO NPA: include param children
        std::set<node_ptr_t> children() const;

        int numChildren() const;

        void exec();


        /* return all parameter values (without querying connected parents)
         */
        std::map<LocalNodeInput, ParamValue> parameters() const;

        /* set a parameter based on a message
         */
        void setParam(boost::shared_ptr<const SetNodeParameterMessage>  m);

        /* set a single parameter value
         */
        template<typename T>
        void setParam(input_id const& p, T const& v) throw(id_error){
            lock_t l(m_inputs_lock);
            private_in_map_t::iterator i = m_inputs.find(p);
            if(i != m_inputs.end()){
                input_ptr ip = i->second;
                // now we can release the inputs lock
                l.unlock();
                if(ip->input_type == InType_Parameter){
                    debug(2) << "param" << p << "set to" << std::boolalpha << v;
                    if(ip->param_value.param.which() == ParamValue(v).which()){
                        ip->param_value = InternalParamValue(v, mkUID(SensorUIDBase::Network));
                        ip->status = NodeInputStatus::New;
                        sendMessage(boost::make_shared<NodeParametersMessage>(
                            m_pl_name, id(), parameters()
                        ));
                        // provide notification that parameters have changed: principally
                        // for asynchronous nodes
                        paramChanged(p);
                        // check to see if the node should be re-scheduled
                        setNewInput(p);
                    }else{
                        error() << p << ":" << v << "has different type to current value" << ip->param_value;
                    }
                }else{
                    error() << p << "is an input not a parameter";
                }
            }else{
                error() << p << "is not a parameter. Parameters:" << parameters();
            }
        }

        /* Derived types overload this for notification of changed parameters:
         * this notification is only useful for asynchronous nodes.
         */
        virtual void paramChanged(input_id const&){ }

        /* return a single parameter value: retrieves value from parent if the
         * parameter is linked to the output of a parent
         */
        template<typename T>
        T param(input_id const& p) const {
            try{
                return getValue<T>(_getAndNotifyIfChangedInternalParam(p).param);
            }catch(boost::bad_get& e){
                error() << p << "is not of requested type:" << e.what();
                throw parameter_error(e.what());
            }
        }

        /* return the parameter value with a specific UID, throws if not
         * possible: use requireSyncInputs, paramAndUID and this function
         * together to get pairs of parameters that are synchronised.
         */
        template<typename T>
        T param(input_id const& p, UID const& uid) const{
            try{
                return
                getValue<T>(_getAndNotifyIfChangedInternalParam(p, &uid).param);
            }catch(boost::bad_get& e){
                error() << p << "is not of requested type:" << e.what();
                throw parameter_error(e.what());
            }
        }

        /* return a single parameter value, and its UID. Retrieves value from parent if the
         * parameter is linked to the output of a parent
         */
        template<typename T>
        std::pair<T, UID> paramAndUID(input_id const& p) const{
            try{
                InternalParamValue val_internal = _getAndNotifyIfChangedInternalParam(p);
                return std::pair<T, UID>(getValue<T>(val_internal.param), val_internal.uid);
            }catch(boost::bad_get& e){
                error() << p << "is not of requested type:" << e.what();
                throw parameter_error(e.what());
            }
        }


        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() const { return false; }

       /* output nodes ignore m_output_demanded: they always execute whenever
        * there is new input.
        */
        virtual bool isOutputNode() const { return false; }

        
        /* !!!! temporary: the location system will be moved to a separate
         * process, at which time img pipeline nodes will no longer be able to
         * receive gps data (this should only ever be used by the slam node
         */
        virtual bool requiresGPS() const { return false; }
        virtual void onGPSLoc(boost::shared_ptr<GPSLocationMessage const>){ }
        virtual bool requiresTelemetry() const { return false; }
        virtual void onTelemetry(boost::shared_ptr<TelemetryMessage const>){ }


        /* Check to see whether this node should be added to the scheduler queue
         */
        enum SchedMode{AllNew, AnyNew, Always, Force};
        void checkAddSched(SchedMode m = AllNew);

    protected:
        /* Get the actual image data associated with an output
         */
        image_ptr_t getOutputImage(output_id const& o_id,
                                   bool suppress_null_warning = false) const throw(id_error);

        /* Get the parameter associated with a parameter output
         */
        InternalParamValue getOutputParam(output_id const& o_id) const throw(id_error);
        
        InternalParamValue getOutputParamWithUID(output_id const& o_id, UID const& uid) const;
        std::set<UID> getOutputParamUIDs(output_id const& o_id) const;
        UID mostRecentUIDOnOutputInSet(output_id const& o_id,  std::set<UID> const& uid_set) const;

        /* Static Constants */
        static const char* Image_In_Name;
        static const char* Image_Out_Name;
        static const char* Image_Out_Copied_Name;

        /* Derived classes override this to do whatever image processing it is
         * that they do.
         */
        virtual void doWork(in_image_map_t&, out_map_t&) = 0;

        /* Priority of this node; this might change dynamically.
         * Used when this node is added to a scheduler queue
         */
        SchedulerPriority m_priority;

        /* How long does this node take to run?
         * used by exec() to decide when to request new input from parents.
         *
         * slow & medium : don't demand output before node execution is complete
         *          fast : demand output from parents before doWork()
         *  asynchronous : don't ever demand output from parents - the node
         *                 will arrange for this to happen via some kind of
         *                 callback magic
         */
        enum Speed {slow=0, medium=5, fast=10, asynchronous=0xff};
        Speed m_speed;

        /* Derived node types should call these functions (probably from their
         * init() method) in order to register valid input output and parameter
         * IDs.
         */
        template<typename T>
        void registerParamID(input_id const& p, T const& default_value,
                             const std::string& tip="",
                             InputSchedType::e const& st = May_Be_Old){
            lock_t l(m_inputs_lock);
            if(m_inputs.count(p)){
                error() << "Duplicate input/parameter id:" << p;
                return;
            }
            m_inputs.insert(private_in_map_t::value_type(
                p, Input::makeParamInputShared(ParamValue(default_value), tip, st)
            ));
            _statusMessage(boost::make_shared<InputStatusMessage>(
                m_pl_name, m_id, p, NodeIOStatus::None
            ));
        }
        
        /* see registerOutputID */
        template<typename T>
        void _explicitRegisterOutputID(output_id const& o, T default_value){
            lock_t l(m_outputs_lock);
            if(m_outputs.count(o))
                throw parameter_error(mkStr() << "Duplicate output id:" << o);

            m_outputs.insert(private_out_map_t::value_type(
                o, boost::make_shared<Output>(output_t(default_value))
            ));

            _statusMessage(boost::make_shared<OutputStatusMessage>(
                m_pl_name, m_id, o, NodeIOStatus::None
            ));
            
        }
        
        /* Template type is used to determine the complete type of the output
         * (image, or ParamValue type) images are specialised, everything
         * else gets shoved into a ParamValue:
         */
        template<typename T>
        void registerOutputID(output_id const& o, T default_value){
            _explicitRegisterOutputID<InternalParamValue>(o,
                InternalParamValue(ParamValue(default_value))
            );
        }
        /* overload for image output type: I'm as shocked as you are that this
         * works
         */
        void registerOutputID(output_id const& o, image_ptr_t default_value=image_ptr_t()){
            _explicitRegisterOutputID<image_ptr_t>(o, default_value);
        }

        void registerInputID(input_id const& i, ConstQualifier isconst, InputSchedType::e const& st = Must_Be_New);

        void requireSyncInputs(input_id const& a, input_id const& b);

        bool unregisterOutputID(output_id const& o, bool warnNonexistent = true);

        void sendMessage(boost::shared_ptr<Message const>, MessageReliability reliability = RELIABLE_MSG) const;
        void subMessage(Message const&);
        //void unsubMessage(boost::shared_ptr<Message const>); !!! TODO

        /* Keep a record of which inputs are new (have changed since they were
         * last used by this node)
         * Check to see if this node should add itself to the scheduler queue
         */
        void setNewInput(input_id const&);
        void setNewInput();
        void clearNewInput();
        /* setNewInput also clears Invalid state */
        void clearValidInput(input_id const&);
        /* checks that image inputs are connected, and have non-NULL values,
         * etc. Requests new input from parents for those that are invalid
         */
        bool ensureValidInput();

        /* All these functions now consider parameters as inputs (but
         * parameters are not Must_Be_New (required) inputs by default
         */
        bool allRequiredInputsAreNew() const; // all includes none
        bool anyRequiredInputsAreNew() const;
        bool anyInputsAreNew() const;

        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void setNewOutputDemanded(output_id const&);
        void clearNewOutputDemanded(output_id const&);
        bool newOutputDemanded() const;

        void setAllowQueue();
        void clearAllowQueue();
        bool allowQueue() const;

        void setExecQueued();
        void clearExecQueued();
        bool execQueued() const;

        // TODO: friends / whatever, and make this protected
    public:
        /* The only derived type that ever needs to call this is ThrottleNode.
         */
        void demandNewParentInput() throw();
    private:
        InternalParamValue _getAndNotifyIfChangedInternalParam(input_id const& p, UID const* uid = NULL) const;

        void demandNewParentInput(input_id const& id) throw();
        
        // synchronised inputs require connected outputs to be queued
        void _pushQueueOutputForSync(output_id const& o_id);
        void _popQueueOutputForSync(output_id const& o_id);

        void _statusMessage(NodeStatus::e const&);
        void _statusMessage(boost::shared_ptr<Message const>);
        static node_id _newID() throw();

        const NodeType::e m_node_type;
        const node_id m_id;

        /* maps an input_id (including parameters) to an output of another node
         */
        private_in_map_t m_inputs;
        mutable mutex_t  m_inputs_lock;

        private_out_map_t m_outputs;
        mutable mutex_t   m_outputs_lock;

        std::set<output_id> m_output_demanded_on;
        mutable mutex_t m_output_demanded_on_lock;

        /** Variables that control when the node is scheduled:
         **/

        /* Prevent checkAddSched from being run concurrently:
         */
        mutable mutex_t m_checking_sched_lock;

        /* prevent nodes (esp. output nodes) from executing in more than one
         * thread at once
         */
        bool m_exec_queued;
        mutable mutex_t m_exec_queued_lock;

        /* generic stop: used by derived types
         */
        bool m_allow_queue;
        mutable mutex_t m_allow_queue_lock;


        /** Other Variables:
         **/

        /* The scheduler associated with this node:
         * This is used by checkAddSched(), which may decide that this node now
         * needs to be executed (this->exec()), so it adds this node to a
         * scheduler queue using:
         *		m_sched.addToQueue(this, m_priority);
         */
        Scheduler& m_sched;

        /* The pipeline manager associated with this node:
         * Used for sending messages, and node pointer -> node id lookups
         */
        ImageProcessor& m_pl;
        const std::string m_pl_name;
        
        /* Derived classes may call stop() in their destructors if they have
         * cleanup that requires execution to be halted first.
         *
         * Currently stop() just performs integrity checks, since by the time a
         * node is being destroyed it is definitely not about to be executed,
         * but in the future it may do other stuff.
         */
        bool m_stopped;
        
        /* keep track of the amount of data being processed */
        cauv::ThroughputCounter m_throughput_counter;
        
        /* don't send status messages too frequently! */
        RateLimiter m_message_throttle;
};

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Node const& n){
    os << "{Node " << &n
       << " type=" << n.type()
       << " id=" << n.id()
       << "}";
    return os;
}

template<typename char_T, typename traits, typename T_ID>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Node::link_target<T_ID> const& l){
    os << "{Link "
       << " node=" << l.node
       << " id=" << l.id
       << "}";
    return os;
}

template<typename cT, typename tT>
std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>& os, Node::InputType const& it){
    switch(it){
        case Node::InType_Image:     return os << "{InputType Image}";
        case Node::InType_Parameter: return os << "{InputType Parameter}";
        default:                     return os << "{InputType ?}";
    }
}

template<typename cT, typename tT>
std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>& os, Node::SchedMode const& m){
    switch(m){
        case Node::AllNew:  return os << "{SchedMode AllNew}";
        case Node::AnyNew:  return os << "{SchedMode AnyNew}";
        case Node::Always:  return os << "{SchedMode Always}";
        case Node::Force:   return os << "{SchedMode Force}";
        default:            return os << "{SchedMode ?}";
    }
    return os;
}

template<typename cT, typename tT>
std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>& os, Node::Input const& i){
    os << "{Input"
       << i.target << i.sched_type << "{" << i.status << "}" << i.input_type << i.tip
       << "}";
    return os;
}

template<typename cT, typename tT>
std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>& os, Node::Output const& o){
    os << "{Output "
       << o.targets << o.value_queue << "demanded=" << std::boolalpha << o.demanded
       << "}";
    return os;
}

template<typename cT, typename tT>
std::basic_ostream<cT, tT>& operator<<(std::basic_ostream<cT, tT>& os, Node::InternalParamValue const& v){
    return os << "{value=" << v.param << " id=" << std::hex << v.uid << "}";
}

} // namespace imgproc
} // namespace cauv

#endif // ndef __NODE_H__
