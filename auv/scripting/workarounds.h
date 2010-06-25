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
#include <stdexcept>

#include <boost/python.hpp>
#include <boost/python/signature.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/tss.hpp>

#include <debug/cauv_debug.h>
#include <common/bash_cout.h>

/** ... but the definitions of the wrappers need boost python (since the whole
 ** point is to wrap with GIL release, so they must be placed after inclusion)
 **/
class GILLockBase: boost::noncopyable{
    public:
        enum lock_release_e{
            Acquire,
            Release
        };
        GILLockBase(lock_release_e const& ar)
            : m_ar(ar){
            if(ar == Acquire)
                acquire();
            else
                release();
        }
        ~GILLockBase(){
            if(m_ar == Acquire)
                release();
            else
                acquire();
        }
    private:
        void release(){
            threadState().push(PyEval_SaveThread());
            debug() << BashColour::Purple << threadState().size()
                    << "PyEval_SaveThread=" << threadState().top();
        }
        void acquire(){
            if(!threadState().size()){
                throw std::runtime_error("aquire() does not correspond to release() on this thread");
            }else{
                debug() << BashColour::Purple << threadState().size()
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

        lock_release_e m_ar;
};

class GILLock: GILLockBase{
    public:
        GILLock()
            : GILLockBase(Acquire){
        }
};

class GILRelease: GILLockBase{
    public:
        GILRelease()
            : GILLockBase(Release){
        }
};

template<class R, class C>
R MemberWrap<R, C>::operator()(C* p){
    GILRelease guard;
    if(m_wrapped){
        debug(4) << "wrap ()";
        return (p->*(this->m_wrapped))();
    }
    debug(4) << "wrap () const";
    assert(this->m_wrapped_const);
    return (p->*(this->m_wrapped_const))();
}

template <class R, class C, class T0>
R MemberWrap<R, C, T0>::operator()(C* p, T0_ p0){
    GILRelease guard;
    debug(4) << "wrap(T0)";
    return (p->*(this->m_wrapped))(p0);
}

template <class R, class C, class T0, class T1>
R MemberWrap<R, C, T0, T1>::operator()(C* p, T0_ p0, T1_ p1){
    GILRelease guard;
    debug(4) << "wrap(T0, T1)";
    return (p->*(this->m_wrapped))(p0, p1);
}

template <class R, class C, class T0, class T1, class T2>
R MemberWrap<R, C, T0, T1, T2>::operator()(C* p, T0_ p0, T1_ p1, T2_ p2){
    GILRelease guard;
    debug(4) << "wrap(T0, T1, T2)";
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

#endif // ndef HACKY_WORKAROUNDS_H

