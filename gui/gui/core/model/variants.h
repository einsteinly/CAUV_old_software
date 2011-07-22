#ifndef GUI_VARIANTS_H
#define GUI_VARIANTS_H

#include <common/image.h>

#include <boost/variant/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/variant/get.hpp>

#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/CameraID.h>

#include <debug/cauv_debug.h>

using namespace std::rel_ops;

namespace cauv {
    namespace gui {

        /// HACK: autopilots aren't ID's in the messages, so we do it ourself here
        namespace AutopilotID {
            enum e {
                Bearing, Pitch, Depth
                    };
        }// namespace AutopilotID


        typedef boost::variant<bool, unsigned int, int, float> numeric_variant_t;
        typedef boost::variant<std::string, MotorID::e, Controller::e, AutopilotID::e, CameraID::e> id_variant_t;
        typedef boost::shared_ptr<const Image> image_variant_t;

        // variant utils

        class id_to_name : public boost::static_visitor<std::string>
        {
        public:
            template <typename T>
                    std::string operator()( T & operand ) const
            {
                std::stringstream str;
                str << operand;
                return str.str();
            }
        };

        // @TODO: put some more specialisations here to make the names a bit prettier
        template <> std::string id_to_name::operator()( AutopilotID::e const& operand ) const;

        class to_float : public boost::static_visitor<float>
        {
        public:
            template <typename T>
                    float operator()( T & operand ) const
            {
                return (float) operand;
            }
        };

        template <class R>
        class from_string: public boost::static_visitor<R>
        {
        public:
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

        class limit_max : public boost::static_visitor<numeric_variant_t>
        {
        public:
            limit_max(const numeric_variant_t & max): m_max(max) {
            }

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                info() << "comparing " << boost::get<T>(m_max) << operand;
                if(boost::get<T>(m_max) < operand) {
                    info() << "too big, returning" << m_max;
                    return m_max;
                } else return numeric_variant_t(operand);
            }
        protected:
            numeric_variant_t m_max;
        };

        class limit_min : public boost::static_visitor<numeric_variant_t>
        {
        public:
            limit_min(const numeric_variant_t & min): m_min(boost::apply_visitor(to_float(), min)) {
            }

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                if(operand < m_min) {
                    return numeric_variant_t((T) m_min);
                } else return numeric_variant_t(operand);
            }
        protected:
            float m_min;
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_VARIANTS_H
