#ifndef GUI_VARIANTS_H
#define GUI_VARIANTS_H

#include <common/image.h>

#include <boost/variant/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/variant/get.hpp>
#include <boost/functional/hash.hpp>
#include <boost/algorithm/string.hpp>

#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/CameraID.h>

#include <debug/cauv_debug.h>

using namespace std::rel_ops;

namespace cauv {
    namespace gui {


        /**
          * Variant definitions
          */

        typedef boost::variant<bool, unsigned int, int, float> numeric_variant_t;
        typedef boost::variant<std::string, MotorID::e, Controller::e, CameraID::e> id_variant_t;
        typedef boost::shared_ptr<const Image> image_variant_t;



        /**
          * Useful variant visitors
          */

        // takes a id_variant_t and converts it to a prettier name than just by using stream operators
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
        // @TODO: put some more specialisations here to make the names a bit prettier


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

        // makes a numeric_variant_t from a string and an example
        // of the type of variant we want to read from the string
        template <class R>
                struct from_string: public boost::static_visitor<R>
        {
            from_string(const std::string & input): m_string(input) {
            }

            template <typename T>
                    R operator()( T & ) const
            {
                return R(boost::lexical_cast<T>(m_string));
            }
        protected:
            const std::string m_string;
        };


        // two clamping visitors, one for max one for min
        struct limit_max : public boost::static_visitor<numeric_variant_t>
        {
            limit_max(const numeric_variant_t & max): m_max(max) {}

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                if(boost::apply_visitor(cast_to<T>(), m_max) < operand) {
                    return boost::apply_visitor(cast_to<T>(), m_max);
                } else return numeric_variant_t(operand);

            }
        protected:
            const numeric_variant_t m_max;
        };

        struct limit_min : public boost::static_visitor<numeric_variant_t>
        {
            limit_min(const numeric_variant_t & min): m_min(min) {}

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                if(boost::apply_visitor(cast_to<T>(), m_min) > operand) {
                    return boost::apply_visitor(cast_to<T>(), m_min);
                } else return numeric_variant_t(operand);
            }
        protected:
            const numeric_variant_t m_min;
        };

        struct wrap : public boost::static_visitor<numeric_variant_t>
        {
            wrap(const numeric_variant_t & min, const numeric_variant_t & max): m_min(min), m_max(max) {}

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                // work out the range
                T min = boost::apply_visitor(cast_to<T>(), m_min);
                T max = boost::apply_visitor(cast_to<T>(), m_max);
                T range = max-min;

                if(operand < min){
                    numeric_variant_t var = numeric_variant_t(operand + range);
                    return boost::apply_visitor(wrap(m_min, m_max), var);
                } else if (operand > max){
                    numeric_variant_t var = numeric_variant_t(operand - range);
                    return boost::apply_visitor(wrap(m_min, m_max), var);
                } else return operand;
            }
        protected:
            const numeric_variant_t m_min;
            const numeric_variant_t m_max;
        };


    } // namespace gui
} // namespace cauv


// extend boost with info about how to hash id_variant_t's
namespace boost {
    inline std::size_t hash_value(const cauv::gui::id_variant_t &variant)
    {
        return boost::apply_visitor(cauv::gui::hash_value(), variant);
    }
}

#endif // GUI_VARIANTS_H
