#ifndef GUI_NUMERICNODE_H
#define GUI_NUMERICNODE_H

#include "../node.h"

#include <QMetaType>

namespace cauv {
    namespace gui {

        /**
          * This class might need some explaining. Because Qt doesn't allow the Q_OBEJCT macro in
          * templated classes (because moc doesn't support it), and because Q_OBJECT is required
          * for signals and slots, such as onSet(whatevertype value) this class is a bit more
          * complicated and messy than it needs to be:
          *
          * numeric_variant_t - basically groups all the types that can be graphed or displayed as
          *                     a number in the data streams list etc... so they can be treated as
          *                     the same type in the rest of the code (avoids lots of tedious duplication)
          *
          * NumericNode - provides all the signals and slots for types found in numeric_variant_t
          *               this means we can still use signals to update various GUI elements. e.g.
          *               onUpdate(float) gets connected to a QComboBox's setValue(float) slot.
          *               BUT because everything is treated as a numeric_variant_t we lose some
          *               type safety
          *
          * TypedNumericNode<T> - restores some of the type safety lost by the variant abstraction.
          *                       largely it's just a wrapper around boost::get<T>(variant)
          *
          */

        class NumericNode : public Node<numeric_variant_t> {
            Q_OBJECT

        protected:
            // to construct use a TypedNumericNode or subclass as it gives some type safety back
            NumericNode(id_variant_t const& id) : Node<numeric_variant_t>(GuiNodeType::NumericNode, id),
            m_maxSet(false), m_minSet(false), m_wraps(false), m_precision(3)
            {
                qRegisterMetaType<numeric_variant_t>("numeric_variant_t");
            }


        public:
            virtual std::string getUnits(){
                return m_units;
            }

            virtual void setUnits(std::string const& units) {
                m_units = units;
            }

            numeric_variant_t numericVariantFromString(std::string const& value){
                numeric_variant_t current = get();
                return boost::apply_visitor(from_string<numeric_variant_t>(value), current);
            }

            virtual bool isMaxSet(){
                return m_maxSet;
            }

            virtual bool isMinSet(){
                return m_minSet;
            }

            virtual bool getWraps() {
                return m_wraps;
            }

            virtual void setWraps(bool wraps){
                m_wraps = wraps;

                Q_EMIT paramsUpdated();
            }

            virtual void setPrecision(unsigned int precision){
                m_precision = precision;

                Q_EMIT paramsUpdated();
            }

            virtual unsigned int getPrecision(){
                return m_precision;
            }


        protected:

            virtual void setMin(numeric_variant_t const& min){
                m_min = min;
                m_minSet = true;

                Q_EMIT paramsUpdated();
            }

            virtual void setMax(numeric_variant_t const& max){
                m_max = max;
                m_maxSet = true;

                Q_EMIT paramsUpdated();
            }

            virtual numeric_variant_t getMaxAsVariant() {
                return m_max;
            }

            virtual numeric_variant_t getMinAsVariant() {
                return m_min;
            }

        public Q_SLOTS:

            virtual void update(numeric_variant_t const& value){
                Node<numeric_variant_t>::update(value);

                switch(value.which()){
                case 0:
                    Q_EMIT onUpdate(boost::get<bool>(value));
                    break;
                case 1:
                    Q_EMIT onUpdate(boost::get<unsigned int>(value));
                    break;
                case 2:
                    Q_EMIT onUpdate(boost::get<int>(value));
                    break;
                case 3:
                    Q_EMIT onUpdate(boost::get<float>(value));
                    Q_EMIT onUpdate((double) boost::get<float>(value));
                    break;
                default:
                    throw std::exception();
                }
            }

            virtual void set_slot(bool value){
                set(value);
            }

            virtual void set_slot(int value){
                set(value);
            }

            virtual void set_slot(unsigned int value){
                set(value);
            }

            virtual void set_slot(float value){
                set(value);
            }

            virtual void set_slot(double value){
                set((float)value);
            }

        protected Q_SLOTS:
            virtual void set(numeric_variant_t const& value){
                numeric_variant_t output = value;
                if(m_wraps){
                    output = boost::apply_visitor(wrap(getMinAsVariant(), getMaxAsVariant()), value);
                }
                if(m_maxSet) {
                    output = boost::apply_visitor(limit_max(m_max), output);
                }
                if(m_minSet) {
                    output = boost::apply_visitor(limit_min(m_min), output);
                }


                Node<numeric_variant_t>::set(output);

                switch(value.which()){
                case 0:
                    Q_EMIT onSet(boost::get<bool>(output));
                    break;
                case 1:
                    Q_EMIT onSet(boost::get<unsigned int>(output));
                    break;
                case 2:
                    Q_EMIT onSet(boost::get<int>(output));
                    break;
                case 3:
                    Q_EMIT onSet(boost::get<float>(output));
                    Q_EMIT onSet((double) boost::get<float>(output));
                    break;
                default:
                    throw std::exception();
                }
            }

        Q_SIGNALS:
            void onUpdate(numeric_variant_t const& value);
            void onUpdate(int value);
            void onUpdate(unsigned int value);
            void onUpdate(float value);
            void onUpdate(double value);
            void onUpdate(bool value);

            void paramsUpdated();

            void onSet(numeric_variant_t const& value);
            void onSet(int value);
            void onSet(unsigned int value);
            void onSet(float value);
            void onSet(double value);
            void onSet(bool value);

        protected:
            std::string m_units;
            numeric_variant_t m_max;
            numeric_variant_t m_min;
            bool m_maxSet, m_minSet, m_wraps;
            unsigned int m_precision;
        };


        // Q_OBJECT classes can't be templated,
        // to add some of the type safety back in it would be nice to
        // have the class templated, so it's done in this subclass
        // bit ugly. i know. (see the longer comment above for more)
        template<class T>
        class TypedNumericNode : public NumericNode {
        public:
            TypedNumericNode(id_variant_t const& id) : NumericNode(id)
            {
                m_value = T();
            }

            T get() {
                try {
                    return boost::get<T>(NumericNode::get());
                } catch (boost::bad_get ex){
                    warning() << ex.what();
                    return T();
                }
            }

            virtual void set(T const& value){
                NumericNode::set(value);
            }

            virtual T getMax() {
                return boost::get<T>(NumericNode::getMaxAsVariant());
            }

            virtual T getMin() {
                return boost::get<T>(NumericNode::getMinAsVariant());
            }

            virtual void setMin(T min){
                NumericNode::setMin(min);
            }

            virtual void setMax(T max){
                NumericNode::setMax(max);
            }
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_NUMERICNODE_H
