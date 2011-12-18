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

#ifndef GUI_VARIANTS_H
#define GUI_VARIANTS_H

#include <boost/variant/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/get.hpp>
#include <boost/functional/hash.hpp>
#include <boost/algorithm/string.hpp>

#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/CameraID.h>

#include <sstream>

using namespace std::rel_ops;

namespace cauv {
    namespace gui {


        /**
          * Variant definitions
          */
        typedef boost::variant<std::string, MotorID::e, Controller::e, CameraID::e, uint32_t> nid_t;

        /**
          * Useful variant visitors
          */

        // takes a nid_t and converts it to a prettier name than just by using stream operators
        class id_to_name : public boost::static_visitor<std::string>
        {
        public:
            template <typename T>
                    std::string operator()( T & operand ) const
            {
                std::stringstream str;
                str << operand;
                std::string name = str.str();
                try {
                    boost::to_lower(name);
                    int index = name.find_last_of(':')+1;
                    return name.substr(index, name.length()-index);
                } catch (std::out_of_range){
                    return name;
                }
            }
        };

        // gets the value of a variant cast to R
        // useful for mixing variants with various Qt classes
        template<class R>
        struct cast_to : public boost::static_visitor<R>
        {
            template <typename T> R operator()( T & operand ) const
            {
                return (R) operand;
            }
        };


        // hashes variants so they can be used as Keys in maps etc...
        struct hash_value : public boost::static_visitor<std::size_t>
        {
            template <typename T> std::size_t operator()( T & operand ) const
            {
                std::size_t seed = 0;
                boost::hash_combine(seed, typeid(T).name());
                boost::hash_combine(seed, operand);
                return seed;
            }
        };

    } // namespace gui
} // namespace cauv


// extend boost with info about how to hash nid_t's
namespace boost {
    inline std::size_t hash_value(const cauv::gui::nid_t &variant)
    {
        return boost::apply_visitor(cauv::gui::hash_value(), variant);
    }
}

#endif // GUI_VARIANTS_H
