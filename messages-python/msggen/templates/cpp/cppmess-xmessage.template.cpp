/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#include "${m.name}Message.h"

\#include <boost/make_shared.hpp>

\#include <utility/string.h>
\#include <utility/serialisation.h>
\#include <debug/cauv_debug.h>

#for $i in $includes
\#include ${i}
#end for

using namespace cauv;

#set $className = $m.name + "Message"
#if $len($m.fields) > 0
cauv::${className}::${className}()
    : Message($m.id, "$g.name"), m_deserialised(true),
      #if $m.numLazyFields() > 0
      m_lazy_fields_deserialised(),
      #end if
      #for i, f in $enumerate($m.fields)
      #if $f.lazy
      m_${f.name}(boost::make_shared< $toCPPType($f.type) >()),
      m_lazy_field_${i}_offset(0),
      #else
      m_${f.name}(),
      #end if
      #end for
      m_bytes(){
}
cauv::${className}::${className}(#slurp
                           #for i, f in $enumerate($m.fields)
#*                        *#$toCPPType($f.type) const& $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                           #end for
#*                        *#)
    : Message($m.id, "$g.name"), m_deserialised(true),
      #if $m.numLazyFields() > 0
      m_lazy_fields_deserialised(),
      #end if
      #for i, f in $enumerate($m.fields)
      #if $f.lazy
      m_${f.name}(boost::make_shared< $toCPPType($f.type) >($f.name)),
      m_lazy_field_${i}_offset(0),
      #else
      m_${f.name}($f.name),
      #end if
      #end for
      m_bytes(){
}
#else
cauv::${className}::${className}() : Message($m.id, "$g.name") { }
#end if

#if $m.numLazyFields() > 0
cauv::${className}::${className}(#slurp
                     #for i, f in $enumerate($m.fields)
#*                  *##if $f.lazy#boost::shared_ptr< $toCPPType($f.type) > const& $f.name#slurp
#*                  *##else#$toCPPType($f.type) const& $f.name#end if##slurp
#*                  *##if $i < $len($m.fields) - 1#, #end if##slurp
                     #end for
#*                  *#)
    : Message($m.id, "$g.name"), m_deserialised(true),
      #if $m.numLazyFields() > 0
      m_lazy_fields_deserialised(),
      #end if
      #for i, f in $enumerate($m.fields)
      #if $f.lazy
      m_${f.name}($f.name),
      m_lazy_field_${i}_offset(0),
      #else
      m_${f.name}($f.name),
      #end if
      #end for
      m_bytes(){
}
#end if

#for i, f in $enumerate($m.fields)
const $toCPPType($f.type)& cauv::$className::${f.name}() const{
    checkDeserialised();
    #if $f.lazy
    // Lazy field: may not be deserialised yet
    if(m_bytes && !m_lazy_fields_deserialised.count($i)){
        cauv::deserialise(m_bytes, m_lazy_field_${i}_offset, *m_${f.name});
        m_lazy_fields_deserialised.insert($i);
        if($m.numLazyFields() == m_lazy_fields_deserialised.size())
            m_bytes.reset();
    }
    return *m_${f.name};
    #else
    return m_${f.name};
    #end if
}
const $toCPPType($f.type)& cauv::$className::get_${f.name}() const{ return ${f.name}(); }
void cauv::$className::${f.name}($toCPPType($f.type) const& $f.name){
    #if $f.lazy
    *m_$f.name = $f.name;
    #else
    m_$f.name = $f.name;
    #end if
}
void cauv::$className::set_${f.name}($toCPPType($f.type) const& ${f.name}_value){ ${f.name}(${f.name}_value); }

#if $f.lazy
void cauv::$className::get_${f.name}_inplace($toCPPType($f.type) &$f.name) const {
    checkDeserialised();
    if (m_bytes && !m_lazy_fields_deserialised.count($i)) {
        cauv::deserialise(m_bytes, m_lazy_field_${i}_offset, ${f.name});
    }
}
#end if

#end for

#if $len($m.fields) > 0
boost::shared_ptr<$className> cauv::$className::fromBytes(const_svec_ptr bytes){
    boost::shared_ptr<$className> ret = boost::make_shared<$className>();
    ret->m_bytes = bytes;
    ret->m_deserialised = false;
    return ret;
}
#else
boost::shared_ptr<$className> cauv::$className::fromBytes(const_svec_ptr){
    boost::shared_ptr<$className> ret = boost::make_shared<$className>();
    return ret;
}
#end if
const_svec_ptr cauv::$className::toBytes() const{
    svec_ptr r = boost::make_shared<svec_t>();
    serialise(r, *this);
    return r;
}
std::string cauv::$className::chil() const{
    #if $len($m.fields)
    checkDeserialised();
    #end if
    return cauv::chil(*this);
}

std::string cauv::${className}::_str() const
{
    std::stringstream ss;
    ss << "$className {";
    #for i, f in $enumerate($m.fields)
    #if hasattr($f.type, "name") and ($f.type.name == "int8" or $f.type.name == "byte") 
    ss << " $f.name = " << (int)${f.name}()#if $i < $len($m.fields) - 1# << ","#end if#;
    #elif hasattr($f.type, "name") and ($f.type.name == "string")
    ss << " $f.name = \"" << ${f.name}()#if $i < $len($m.fields) - 1# << "\","#else# << "\""#end if#;
    #else
    ss << " $f.name = " << ${f.name}()#if $i < $len($m.fields) - 1# << ","#end if#;
    #end if
    #end for
    ss << " }";
    return ss.str();
}

#if $m.numLazyFields() > 0
template<typename T>
static void serialiseLazyField(cauv::svec_ptr p, T const& v){
    uint32_t initial_size = p->size();
    cauv::serialise(p, uint32_t(0));
    cauv::serialise(p, v);
    *reinterpret_cast<uint32_t*>(&(*p)[initial_size]) = p->size() - initial_size;
}
#end if
#if len($m.fields) > 0
void cauv::serialise(svec_ptr p, $className const& v){
#else
void cauv::serialise(svec_ptr p, $className const&){
#end if
    ## reserve estimated minimum message size
    p->reserve(#echo 4 *(1 + len($m.fields))#);
    ## message type
    cauv::serialise(p, uint32_t($m.id));
    cauv::serialise(p, uint32_t($hex($m.check_hash)));
    #for f in $m.fields
    #if $f.lazy
    serialiseLazyField(p, *v.m_$f.name);
    #else
    cauv::serialise(p, v.m_$f.name);
    #end if
    #end for
}
void cauv::$className::deserialise() const{
    ## message ID + hash comes first
    #if len($m.fields) > 0
    uint32_t offset = 8;
    #end if
    #if $m.numLazyFields() > 0
    uint32_t skip = 0;
    m_lazy_fields_deserialised.clear();
    #end if
    #for i, f in $enumerate($m.fields)
    #if $f.lazy
    m_lazy_field_${i}_offset = offset + cauv::deserialise(m_bytes, offset, skip);
    offset += skip;
    #else
    offset += cauv::deserialise(m_bytes, offset, m_${f.name});
    #end if
    #end for
    #if $m.numLazyFields() == 0
    m_bytes.reset();
    #end if
}
#if len($m.fields) > 0
std::string cauv::chil($className const& v){
#else
std::string cauv::chil($className const&){
#end if
    std::string r = "${m.id}(";
    #for i,f in $enumerate($m.fields)
        #if $f.lazy
            ## make sure lazy fields are deserialised!
            #if i+1 != len($m.fields)
    r += cauv::chil(v.get_${f.name}()) + ",";
            #else
    r += cauv::chil(v.get_${f.name}());
            #end if
        #else
            #if i+1 != len($m.fields)
    r += cauv::chil(v.m_${f.name}) + ",";
            #else
    r += cauv::chil(v.m_${f.name});
            #end if
        #end if
    #end for
    return r + ")";
}
