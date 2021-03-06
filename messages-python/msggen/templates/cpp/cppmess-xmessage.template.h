/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${m.name.upper()}_MESSAGE_H__
\#define __CAUV_${m.name.upper()}_MESSAGE_H__

\#include "message.h"

#if $m.numLazyFields() > 0
\#include <set>
#end if

\#include <boost/shared_ptr.hpp>

#for $i in $includes
\#include ${i}
#end for

namespace cauv{

#set $className = $m.name + "Message"
class $className : public Message
{
    public:
        ${className}();
        #if $len($m.fields) > 0
        ${className}(#slurp
                     #for i, f in $enumerate($m.fields)
#*                  *#$toCPPType($f.type) const& $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                     #end for
#*                  *#);
        #end if
        #if $m.numLazyFields() > 0
        ${className}(#slurp
                     #for i, f in $enumerate($m.fields)
#*                  *##if $f.lazy#boost::shared_ptr< $toCPPType($f.type) > const& $f.name#slurp
#*                  *##else#$toCPPType($f.type) const& $f.name#end if##slurp
#*                  *##if $i < $len($m.fields) - 1#, #end if##slurp
                     #end for
#*                  *#);
        #end if

        #for $f in $m.fields
        const $toCPPType($f.type)& ${f.name}() const;
        const $toCPPType($f.type)& get_${f.name}() const;
        void ${f.name}($toCPPType($f.type) const& $f.name);
        void set_${f.name}($toCPPType($f.type) const& ${f.name}_value);
        #if $f.lazy
        void get_${f.name}_inplace($toCPPType($f.type)& $f.name) const;
        #end if
        
        #end for

        static boost::shared_ptr<${className}> fromBytes(const_svec_ptr bytes);
        virtual const_svec_ptr toBytes() const;
        virtual std::string chil() const;

    protected:
        virtual std::string _str() const; 
    
    private:
        void deserialise() const;
        inline void checkDeserialised() const{
            if(!m_deserialised){
                deserialise();
                m_deserialised = true;
            }
        }
        mutable bool m_deserialised;

    #if $m.numLazyFields() > 0
        mutable std::set<int> m_lazy_fields_deserialised;
    #end if

    #for i, f in $enumerate($m.fields)
        #if $f.lazy
        mutable boost::shared_ptr< $toCPPType($f.type) > m_$f.name;
        mutable uint32_t m_lazy_field_${i}_offset;
        #else
        mutable $toCPPType($f.type) m_$f.name;
        #end if
    #end for

        mutable const_svec_ptr m_bytes;

    friend void serialise(svec_ptr, $className const&);
    friend std::string chil($className const&);
};

void serialise(svec_ptr p, $className const&);
std::string chil($className const&);

} // namespace cauv

\#endif // __CAUV_${m.name.upper()}_MESSAGE_H__
