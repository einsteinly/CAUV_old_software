/* Portions of this file are Copyright 2011 Cambridge Hydronautics Ltd.
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
 *
 * The code that this does not apply to is marked with comments and
 * attribution, below.
 */

#ifndef HACKY_WORKAROUNDS_H
#define HACKY_WORKAROUNDS_H
/*** This file contains various workarounds for doing crazy things with boost
 *** python, like wrapping member functions inside function objects that
 *** release the Global Interpreter Lock, and passing around shared_ptrs to
 *** const values.
 ***
 *** Reading this file is not recommended for preserving sanity.
 ***/

#include <boost/mpl/vector.hpp>
#include <boost/type_traits.hpp>

/** for some reason, get_signature overloads must be implemented before the
 ** inclusion of boost/python
 **/

// heavily borrowing from http://stackoverflow.com/questions/2135457/how-to-write-a-wrapper-over-functions-and-member-functions-that-executes-some-cod
template <class R, class C, class T0=void, class T1=void, class T2=void, class T3=void>
struct MemberWrap;

template<class R, class C>
struct MemberWrap<R, C>{
    typedef R (C::*member_t)();
    typedef R (C::*const_member_t)() const;
    
    // can't const-cast member functions...
    MemberWrap(member_t f) : m_wrapped(f), m_wrapped_const(){ }
    MemberWrap(const_member_t f) : m_wrapped(), m_wrapped_const(f){ }
    R operator()(C* p);
    
    member_t m_wrapped;
    const_member_t m_wrapped_const;
};

template <class R, class C, class T0>
struct MemberWrap<R, C, T0>{
    typedef R (C::*member_t)(T0);
    typedef typename boost::add_reference<T0>::type T0_;

    MemberWrap(member_t f) : m_wrapped(f){ }
    R operator()(C* p, T0_ p0);
    
    member_t m_wrapped;
};

template <class R, class C, class T0, class T1>
struct MemberWrap<R, C, T0, T1>{
    typedef R (C::*member_t)(T0, T1);
    typedef typename boost::add_reference<T0>::type T0_;
    typedef typename boost::add_reference<T1>::type T1_;

    MemberWrap(member_t f) : m_wrapped(f){ }
    R operator()(C* p, T0_ p0, T1_ p1);
    
    member_t m_wrapped;
};

template <class R, class C, class T0, class T1, class T2>
struct MemberWrap<R, C, T0, T1, T2>{
    typedef R (C::*member_t)(T0, T1, T2);
    typedef typename boost::add_reference<T0>::type T0_;
    typedef typename boost::add_reference<T1>::type T1_;
    typedef typename boost::add_reference<T2>::type T2_;

    MemberWrap(member_t f) : m_wrapped(f){ }
    R operator()(C* p, T0_ p0, T1_ p1, T2_ p2);
    
    member_t m_wrapped;
};


namespace bm = boost::mpl;

namespace boost{
namespace python{
namespace detail{

template <class R, class C>
inline bm::vector<R, C*> get_signature(MemberWrap<R, C>, void* = 0){ return bm::vector<R, C*>(); }
template <class R, class C, class T0>
inline bm::vector<R, C*, T0> get_signature(MemberWrap<R, C, T0>, void* = 0){ return bm::vector<R, C*, T0>(); }
template <class R, class C, class T0, class T1>
inline bm::vector<R, C*, T0, T1> get_signature(MemberWrap<R, C, T0, T1>, void* = 0){ return bm::vector<R, C*, T0, T1>(); }
template <class R, class C, class T0, class T1, class T2>
inline bm::vector<R, C*, T0, T1, T2> get_signature(MemberWrap<R, C, T0, T1, T2>, void* = 0){ return bm::vector<R, C*, T0, T1, T2>(); }
template <class R, class C, class T0, class T1, class T2, class T3>
inline bm::vector<R, C*, T0, T1, T2, T3> get_signature(MemberWrap<R, C, T0, T1, T2, T3>, void* = 0){ return bm::vector<R, C*, T0, T1, T2, T3>(); }

} // namespace detail
} // namespace python
} // namespace boost

template <class R, class C> MemberWrap<R, C> wrap(R (C::*p)()){ return MemberWrap<R, C>(p); }
template <class R, class C> MemberWrap<R, C> wrap(R (C::*p)() const){ return MemberWrap<R, C>(p); } 
template <class R, class C, class T0> MemberWrap<R, C, T0> wrap(R (C::*p)(T0)){ return MemberWrap<R, C, T0>(p); } 
template <class R, class C, class T0, class T1> MemberWrap<R, C, T0, T1> wrap(R (C::*p)(T0, T1)){ return MemberWrap<R, C, T0, T1>(p); } 
template <class R, class C, class T0, class T1, class T2> MemberWrap<R, C, T0, T1, T2> wrap(R (C::*p)(T0, T1, T2)){ return MemberWrap<R, C, T0, T1, T2>(p); }

#include <stack>
//#include <stdexcept>

#include <boost/python.hpp>
#include <boost/python/signature.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/tss.hpp>

#include <debug/cauv_debug.h>
#include <utility/bash_cout.h>

/** ... but the definitions of the wrappers need boost python (since the whole
 ** point is to wrap with GIL release, so they must be placed after inclusion)
 **/
class ThreadSaveRestoreBase: boost::noncopyable{
    public:
        enum lock_release_e{
            Acquire,
            Release
        };
        ThreadSaveRestoreBase(lock_release_e const& ar)
            : m_ar(ar){
            assert(PyEval_ThreadsInitialized());
            if(ar == Acquire)
                acquire();
            else
                release();
        }
        ~ThreadSaveRestoreBase(){
            if(m_ar == Acquire)
                release();
            else
                acquire();
        }
    private:
        void release() const{
            // if this is crashing, check that only one python version is
            // linked: multiple linked python libraries can cause NULL tstate
            // errors
            threadState().push(PyEval_SaveThread());
            debug(9) << BashColour::Purple << threadState().size()
                     << "PyEval_SaveThread=" << threadState().top();
        }
        void acquire() const{
            if(!threadState().size()){
                // but don't throw an error, since that would probably cause
                // indecipherable errors instead of something that might lead
                // someone to the solution of whatever caused this
                error() << BashColour::Red << "probably FATAL: "
                        << "acquire() does not correspond to release() on this thread";
            }else{
                debug(9) << BashColour::Purple << threadState().size()
                        << "PyEval_RestoreThread(" << threadState().top() << ")";
                PyEval_RestoreThread(threadState().top());
                threadState().pop();
            }
        }
        static std::stack<PyThreadState*>& threadState(){
            static boost::thread_specific_ptr< std::stack<PyThreadState*> > save;
            if(!save.get())
                save.reset(new std::stack<PyThreadState*>);
            return *save.get();
        }

        const lock_release_e m_ar;
};

class ThreadRestore: ThreadSaveRestoreBase{
    public:
        ThreadRestore()
            : ThreadSaveRestoreBase(Acquire){
        }
};

class ThreadSave: ThreadSaveRestoreBase{
    public:
        ThreadSave()
            : ThreadSaveRestoreBase(Release){
        }
};

class GILLock: boost::noncopyable{
    public:
        GILLock()
            : m_released(false), m_gs(PyGILState_Ensure()){
            debug(9) << BashColour::Green << "PyGILState_Ensure=" << m_gs;
        }
        void release(){
            if(m_released)
                return;
            debug(9) << BashColour::Green << "PyGILState_Release(" << m_gs << ")";
            PyGILState_Release(m_gs);
            m_released = true;
        }
        ~GILLock(){
            release();
        }
    private:
        bool m_released;
        PyGILState_STATE m_gs;
};

template<class R, class C>
R MemberWrap<R, C>::operator()(C* p){
    ThreadSave guard;
    if(m_wrapped){
        debug(11) << "wrap ()";
        return (p->*(this->m_wrapped))();
    }
    debug(11) << "wrap () const";
    assert(this->m_wrapped_const);
    return (p->*(this->m_wrapped_const))();
}

template <class R, class C, class T0>
R MemberWrap<R, C, T0>::operator()(C* p, T0_ p0){
    ThreadSave guard;
    debug(11) << "wrap(T0)";
    return (p->*(this->m_wrapped))(p0);
}

template <class R, class C, class T0, class T1>
R MemberWrap<R, C, T0, T1>::operator()(C* p, T0_ p0, T1_ p1){
    ThreadSave guard;
    debug(11) << "wrap(T0, T1)";
    return (p->*(this->m_wrapped))(p0, p1);
}

template <class R, class C, class T0, class T1, class T2>
R MemberWrap<R, C, T0, T1, T2>::operator()(C* p, T0_ p0, T1_ p1, T2_ p2){
    ThreadSave guard;
    debug(11) << "wrap(T0, T1, T2)";
    return (p->*(this->m_wrapped))(p0, p1, p2);
}

/** .. End of all the workaround stuff
 **/


/** I lied, here is some more evil hackery for boost::shared_ptr<T const>
 ** pointer conversions, see
 ** http://language-binding.net/pyplusplus/troubleshooting_guide/shared_ptr/shared_ptr.html
 **/
namespace boost{

template<class T>
inline T* get_pointer( boost::shared_ptr<const T> const& p ){
    return const_cast< T* >( p.get() );
}

namespace python{

template<class T>
struct pointee< boost::shared_ptr<T const> >{
    typedef T type;
};

} // namespace python
} // namespace boost

template< class T >
void register_shared_ptrs_to_python(){
    namespace bp = boost::python;
    bp::register_ptr_to_python< boost::shared_ptr< T > >();
    bp::register_ptr_to_python< boost::shared_ptr< const T > >();
    bp::implicitly_convertible< boost::shared_ptr< T >, boost::shared_ptr< const T > >();
}

/** now really end all evil hackery
 **/





/* Actually no, some evil hackery for bool friendly isinstance.
 * ints can convert to bools, but we don't want that, so explicitly
 * forbid it.
 */
template<typename T>
inline bool isinstance(const boost::python::object& o)
{
    boost::python::extract<T> x(o);
    return x.check();
}

// Lots of stuff is extractable to bool, so specialise it to actually
// use isinstance
template<>
inline bool isinstance<bool>(const boost::python::object& o)
{
    using namespace boost::python;

    try {
        object main = import("__main__");
        object main_namespace = main.attr("__dict__");
        
        dict globals;
        globals["o"] = o;
        object ret = eval(
            "isinstance(o, bool)",
            main_namespace, globals); 
        return extract<bool>(ret);
    }
    catch (error_already_set const&) { 
        PyErr_Print(); 
        return false;
    } 
}

// A specialisations for the float types
inline bool isfloat(const boost::python::object& o)
{
    using namespace boost::python;

    try {
        object main = import("__main__");
        object main_namespace = main.attr("__dict__");
        
        dict globals;
        globals["o"] = o;
        object ret = eval(
            "isinstance(o, float)",
            main_namespace, globals); 
        return extract<bool>(ret);
    }
    catch (error_already_set const&) { 
        PyErr_Print(); 
        return false;
    } 
}
template<>
inline bool isinstance<float>(const boost::python::object& o) { return isfloat(o); }
template<>
inline bool isinstance<double>(const boost::python::object& o) { return isfloat(o); }

// A bunch of specialisations for the bajillion int types
inline bool isint(const boost::python::object& o)
{
    using namespace boost::python;

    try {
        object main = import("__main__");
        object main_namespace = main.attr("__dict__");
        
        dict globals;
        globals["o"] = o;
        object ret = eval(
            "(isinstance(o, int) or isinstance(o, long)) and not isinstance(o, bool)",
            main_namespace, globals); 
        return extract<bool>(ret);
    }
    catch (error_already_set const&) { 
        PyErr_Print(); 
        return false;
    } 
}
template<>
inline bool isinstance<char>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<unsigned char>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<short>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<unsigned short>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<int>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<unsigned int>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<long>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<unsigned long>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<long long>(const boost::python::object& o) { return isint(o); }
template<>
inline bool isinstance<unsigned long long>(const boost::python::object& o) { return isint(o); }


/*
 * Shamelessly stolen from cctbx
 * cctbx.sourceforge.net (probably BSD Licensed.. some time we should make a
 * proper list of all the license attributions we need)
 */
struct default_policy
{
    static bool check_convertibility_per_element() { return true; }

    template <typename ContainerType>
    static bool check_size(boost::type<ContainerType>, std::size_t) { return true; }

    template <typename ContainerType>
    static void assert_size(boost::type<ContainerType>, std::size_t) {}

    template <typename ContainerType>
    static void reserve(ContainerType&, std::size_t) {}
};
struct variable_capacity_policy : default_policy
{
    template <typename ContainerType>
    static void reserve(ContainerType& a, std::size_t sz)
    {
        a.reserve(sz);
    }

    template <typename ContainerType, typename ValueType>
    static void set_value(
            ContainerType& a,
#if !defined(NDEBUG)
            std::size_t i,
#else
            std::size_t,
#endif
            ValueType const& v)
    {
        assert(a.size() == i);
        a.push_back(v);
    }
};
struct variable_capacity_map_policy : default_policy
{
    template <typename ContainerType, typename ValueType>
    static void set_value(
            ContainerType& a,
#if !defined(NDEBUG)
            std::size_t i,
#else
            std::size_t,
#endif
            ValueType const& v)
    {
        assert(a.size() == i);
        a.insert(v);
    }
};
template <typename ContainerType, typename ConversionPolicy>
struct from_python_sequence
{
    typedef typename ContainerType::value_type container_element_type;

    from_python_sequence()
    {
        boost::python::converter::registry::push_back(
                &convertible,
                &construct,
                boost::python::type_id<ContainerType>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!(PyList_Check(obj_ptr)
                    || PyTuple_Check(obj_ptr)
                    || PyIter_Check(obj_ptr)
                    || PyRange_Check(obj_ptr)
                    || (   !PyString_Check(obj_ptr)
                        && !PyUnicode_Check(obj_ptr)
                        && (   obj_ptr->ob_type == 0
                            || obj_ptr->ob_type->ob_type == 0
                            || obj_ptr->ob_type->ob_type->tp_name == 0
                            || std::strcmp(
                                obj_ptr->ob_type->ob_type->tp_name,
                                "Boost.Python.class") != 0)
                        && PyObject_HasAttrString(obj_ptr, "__len__")
                        && PyObject_HasAttrString(obj_ptr, "__getitem__")))) return 0;
        boost::python::handle<> obj_iter(
                boost::python::allow_null(PyObject_GetIter(obj_ptr)));
        if (!obj_iter.get()) { // must be convertible to an iterator
            PyErr_Clear();
            return 0;
        }
        if (ConversionPolicy::check_convertibility_per_element()) {
            int obj_size = PyObject_Length(obj_ptr);
            if (obj_size < 0) { // must be a measurable sequence
                PyErr_Clear();
                return 0;
            }
            if (!ConversionPolicy::check_size(
                        boost::type<ContainerType>(), obj_size)) return 0;
            bool is_range = PyRange_Check(obj_ptr);
            size_t i=0;
            if (!all_elements_convertible(obj_iter, is_range, i)) return 0;
            if (!is_range) assert(i == (size_t)obj_size);
        }
        return obj_ptr;
    }

    // This loop factored out by Achim Domma to avoid Visual C++
    // Internal Compiler Error.
    static bool all_elements_convertible(
                    boost::python::handle<>& obj_iter,
                    bool is_range,
                    std::size_t& i)
    {
        for(;;i++) {
            boost::python::handle<> py_elem_hdl(
                    boost::python::allow_null(PyIter_Next(obj_iter.get())));
            if (PyErr_Occurred()) {
                PyErr_Clear();
                return false;
            }
            if (!py_elem_hdl.get())
                break; // end of iteration
            boost::python::object py_elem_obj(py_elem_hdl);
            boost::python::extract<container_element_type>
                elem_proxy(py_elem_obj);
            if (!elem_proxy.check())
                return false;
            if (is_range)
                break; // in a range all elements are of the same type
        }
        return true;
    }

    static void construct(
            PyObject* obj_ptr,
            boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        boost::python::handle<> obj_iter(PyObject_GetIter(obj_ptr));
        void* storage = (
                (boost::python::converter::rvalue_from_python_storage<ContainerType>*)
                data)->storage.bytes;
        new (storage) ContainerType();
        data->convertible = storage;
        ContainerType& result = *((ContainerType*)storage);
        std::size_t i=0;
        for(;;i++) {
            boost::python::handle<> py_elem_hdl(
                    boost::python::allow_null(PyIter_Next(obj_iter.get())));
            if (PyErr_Occurred()) boost::python::throw_error_already_set();
            if (!py_elem_hdl.get()) break; // end of iteration
            boost::python::object py_elem_obj(py_elem_hdl);
            boost::python::extract<container_element_type> elem_proxy(py_elem_obj);
            ConversionPolicy::set_value(result, i, elem_proxy());
        }
        ConversionPolicy::assert_size(boost::type<ContainerType>(), i);
    }
};
template <typename MapType, typename ConversionPolicy>
struct from_python_dict
{
    typedef typename MapType::value_type v_t;
    typedef typename MapType::key_type k_t;
    typedef typename MapType::mapped_type m_t;

    from_python_dict()
    {
        boost::python::converter::registry::push_back(
                &convertible,
                &construct,
                boost::python::type_id<MapType>()
#ifdef BOOST_PYTHON_SUPPORTS_PY_SIGNATURES
                , &boost::python::converter::wrap_pytype<&PyDict_Type>::get_pytype
#endif
                );
    }

    static void* convertible(PyObject* obj_ptr)
    {
        debug(15) << "map convertible start";
        if(!PyDict_Check(obj_ptr))
            return 0;
        debug(15) << "map convertible dict ok";
        boost::python::handle<> obj_hdl(boost::python::borrowed(obj_ptr));
        boost::python::object obj_obj(obj_hdl);
        boost::python::extract<boost::python::dict> obj_proxy(obj_obj);
        if (!obj_proxy.check())
            return 0;
        boost::python::dict obj_dict = obj_proxy();
       
        debug(15) << "map convertible proxy ok";

        if (ConversionPolicy::check_convertibility_per_element()) {
            if (!ConversionPolicy::check_size(boost::type<MapType>(), boost::python::len(obj_dict)))
                return 0;
        debug(15) << "map convertible size ok";
            if (!all_elements_convertible(obj_dict))
                return 0;
        debug(15) << "map convertible elms ok";
        }

        return obj_ptr;
    }
    
    static bool all_elements_convertible(
                    boost::python::dict& obj_dict)
    {
        boost::python::list keys = obj_dict.keys();
        int len_keys = boost::python::len(keys);
        debug(15) << "map convertible elms";
        for(int i=0;i<len_keys;i++) {
        debug(15) << "map convertible elm"<<i;
            boost::python::object key_obj = keys[i];
            boost::python::extract<k_t> key_proxy(key_obj);
            if (!key_proxy.check()) {
                return false;
            }
        debug(15) << "map convertible elm"<<i<<"key ok";
            boost::python::object value_obj = obj_dict[key_obj];
            boost::python::extract<m_t> value_proxy(value_obj);
            if (!value_proxy.check()) {
                return false;
            }
        debug(15) << "map convertible elm"<<i<<"val ok";
        }
        return true;
    }


    static void construct(
            PyObject* obj_ptr,
            boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        boost::python::handle<> obj_hdl(boost::python::borrowed(obj_ptr));
        boost::python::object obj_obj(obj_hdl);
        boost::python::extract<boost::python::dict> obj_proxy(obj_obj);
        boost::python::dict other = obj_proxy();
        void* storage = (
                (boost::python::converter::rvalue_from_python_storage<MapType>*)
                data)->storage.bytes;
        new (storage) MapType();
        data->convertible = storage;
        MapType& self = *((MapType*)storage);
        boost::python::list keys = other.keys();
        int len_keys = boost::python::len(keys);
        for(int i=0;i<len_keys;i++) {
            boost::python::object key_obj = keys[i];
            boost::python::extract<k_t> key_proxy(key_obj);
            if (!key_proxy.check()) {
                PyErr_SetString(PyExc_KeyError, "Unsuitable type.");
                boost::python::throw_error_already_set();
            }
            boost::python::object value_obj = other[key_obj];
            boost::python::extract<m_t> value_proxy(value_obj);
            if (!value_proxy.check()) {
                PyErr_SetString(PyExc_ValueError, "Unsuitable type.");
                boost::python::throw_error_already_set();
            }
            k_t key = key_proxy();
            m_t value = value_proxy();
            
            ConversionPolicy::set_value(self, i, v_t(key, value));
        }
    }
};
template <typename VariantType, typename WrapperType, typename ValueType>
struct from_python_variant
{
    from_python_variant()
    {
        boost::python::converter::registry::push_back(
                &convertible,
                &construct,
                boost::python::type_id<VariantType>()
                );
    }

    static void* convertible(PyObject* obj_ptr)
    {
        debug(3) << "variant convertible";
        boost::python::handle<> obj_hdl(boost::python::borrowed(obj_ptr));
        boost::python::object obj_obj(obj_hdl);
        if(!isinstance<ValueType>(obj_obj))
            return 0;
        debug(3) << "variant convertible isinstance ok";
        return obj_ptr;
    }
    
    static void construct(
            PyObject* obj_ptr,
            boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        boost::python::handle<> obj_hdl(boost::python::borrowed(obj_ptr));
        boost::python::object obj_obj(obj_hdl);
        void* storage = (
                (boost::python::converter::rvalue_from_python_storage<WrapperType>*)
                data)->storage.bytes;
        
        boost::python::extract<ValueType> val_proxy(obj_obj);
        new (storage) WrapperType(static_cast<ValueType>(val_proxy()));
        
        data->convertible = storage;
    }
};





#endif // ndef HACKY_WORKAROUNDS_H

