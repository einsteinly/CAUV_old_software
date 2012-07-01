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

#include <QVariant>
#include <QColor>

#include <boost/mpl/pop_front.hpp>

#include <boost/variant/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/get.hpp>
#include <boost/functional/hash.hpp>
#include <boost/algorithm/string.hpp>

#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>
#include <generated/types/CameraID.h>
#include <common/msg_classes/colour.h>

#include <sstream>

#include <debug/cauv_debug.h>

using namespace std::rel_ops;

namespace cauv {
    namespace gui {

        struct TypedQColor : public QColor {
        public:
            TypedQColor():
                QColor(){
            }

            TypedQColor(const QColor &color):
                QColor(color){
            }

            virtual ColourType::e colorType() const {
                return m_type;
            }

            virtual void setColorType(ColourType::e t) {
                m_type = t;
            }

            ColourType::e m_type;
        };

        Colour qColorToColour(TypedQColor colour);

        TypedQColor colorToQColour(Colour const& colour);

        //
        // helper functions to convert between boost variants and QVariants
        // boost::variant to QVariant
        //
        struct variantToQVariantVisitor : boost::static_visitor<QVariant> {

            QVariant operator()( std::string const& str ) const {
                return QString::fromStdString( str );
            }

            QVariant operator()( Colour const& colour ) const {
                return QVariant(colorToQColour( colour ));
            }

            template <typename T>
            QVariant operator()( const T & operand ) const {
                return qVariantFromValue( operand );
            }
        };

        template <class T>
        QVariant variantToQVariant(T boostVariant){
            return boost::apply_visitor(variantToQVariantVisitor(), boostVariant);
        }

        //
        // QVariant to boost::variant
        // (meta-programming, sorry)
        //
        namespace mpl = boost::mpl;

        template <typename T_Variant, typename Types>
        typename boost::enable_if< mpl::empty<Types>, T_Variant >::type
        qVariantToVariant_helper( const QVariant & ) {
            throw std::bad_cast();
        }

        template <typename T_Variant, typename Types>
        typename boost::disable_if< mpl::empty<Types>, T_Variant >::type
        qVariantToVariant_helper( const QVariant & qv ) {
            typedef typename mpl::front<Types>::type Head;
            typedef typename mpl::pop_front<Types>::type Tail;

            //info() << qv.userType() << "=" << qMetaTypeId<Head>();
            if(qv.userType() == qMetaTypeId<Head>()){
                //info() << qv.toString().toStdString() << "when cast" << qv.value<Head>();
                return T_Variant( qv.value<Head>() );
            }else{
                return qVariantToVariant_helper<T_Variant,Tail>( qv );
            }
        }

        template <typename T_Variant>
        T_Variant qVariantToVariant( const QVariant & qv ) {
            //info() << "recieved qvariant: " << qv.toString().toStdString();
            if(qv.userType() == qMetaTypeId<QString>()){
                // QStrings get converted to std::strings on the way out:
                return T_Variant(qv.value<QString>().toStdString());
            } else {
                return qVariantToVariant_helper<T_Variant,typename T_Variant::types>( qv );
            }
        }


        /**
          * Variant definitions
          */
        typedef boost::variant<std::string, MotorID::e, Controller::e, CameraID::e, uint32_t, int32_t> nid_t;

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
