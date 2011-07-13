#ifndef GUI_VARIANTS_H
#define GUI_VARIANTS_H

#include <common/image.h>

#include <boost/variant/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>

using namespace std::rel_ops;

namespace cauv {
    namespace gui {

        typedef boost::variant<bool, unsigned int, int, float> numeric_variant_t;
        typedef boost::shared_ptr<const Image> image_variant_t;

        // variant utils
        class to_float : public boost::static_visitor<float>
        {
        public:
            template <typename T>
                    float operator()( T & operand ) const
            {
                return (float) operand;
            }
        };

        class limit_max : public boost::static_visitor<numeric_variant_t>
        {
        public:
            limit_max(const numeric_variant_t & max): m_max(boost::apply_visitor(to_float(), max)) {
            }

            template <typename T> numeric_variant_t operator()( T & operand ) const {
                if(operand > m_max) {
                    return numeric_variant_t((T) m_max);
                } else return numeric_variant_t(operand);
            }
        protected:
            float m_max;
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
