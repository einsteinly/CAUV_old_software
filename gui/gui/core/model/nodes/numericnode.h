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

#ifndef GUI_NUMERICNODE_H
#define GUI_NUMERICNODE_H

#include "../node.h"

#include <QMetaType>

namespace cauv {
    namespace gui {

        class NumericNodeBase : public Node {
            Q_OBJECT

        protected:
            NumericNodeBase(nid_t const& id) : Node(GuiNodeType::NumericNode, id),
            m_maxSet(false), m_minSet(false), m_wraps(false), m_precision(3)
            {
            }


        public:
            std::string getUnits(){
                return m_units;
            }

            void setUnits(std::string const& units) {
                m_units = units;
            }

            bool isMaxSet(){
                return m_maxSet;
            }

            bool isMinSet(){
                return m_minSet;
            }

            bool getWraps() {
                return m_wraps;
            }

            void setWraps(bool wraps){
                m_wraps = wraps;

                Q_EMIT paramsUpdated();
            }

            void setPrecision(unsigned int precision){
                m_precision = precision;

                Q_EMIT paramsUpdated();
            }

            unsigned int getPrecision(){
                return m_precision;
            }

            void setMin(QVariant const& min){
                m_min = min;
                m_minSet = true;

                Q_EMIT paramsUpdated();
            }

            void setMax(QVariant const& max){
                m_max = max;
                m_maxSet = true;

                Q_EMIT paramsUpdated();
            }

        Q_SIGNALS:
            void paramsUpdated();

        protected:
            std::string m_units;
            QVariant m_max;
            QVariant m_min;
            bool m_maxSet, m_minSet, m_wraps;
            unsigned int m_precision;
        };


        template<class T>
        class NumericNode : public NumericNodeBase {
        public:
            NumericNode(nid_t const& id) : NumericNodeBase(id)
            {
                m_value = QVariant(T());
            }

            T get() {
                return m_value.value<T>();
            }

            virtual T wrap(T const& value){
                // work out the range
                T range = getMax()-getMin();

                if(value < getMin()){
                    T var = value + range;
                    return wrap(var);
                } else if (value > getMax()){
                    T var = value - range;
                    return wrap(var);
                } else return value;
            }


            virtual bool set(QVariant const& value){
                return set(value.value<T>());
            }

            virtual bool set(T const& value){
                debug() << "NumericNode::set() cleaning value";

                // validate the input as this has come from
                // a user.

                T cleanVal = value;

                // first check if the data wraps
                if (getWraps())
                    cleanVal = wrap(value);
                // if not then maybe it's clamped?
                else {
                    // upper bound check
                    if(isMaxSet() && cleanVal > getMax())
                        cleanVal = getMax();

                    // lower bound check
                    if(isMinSet() && cleanVal < getMin())
                        cleanVal = getMin();
                }

                return NumericNodeBase::set(cleanVal);
            }

            virtual T getMax() {
                return m_max.value<T>();
            }

            virtual T getMin() {
                return m_min.value<T>();
            }

            virtual void setMin(T min){
                NumericNodeBase::setMin(min);
            }

            virtual void setMax(T max){
                NumericNodeBase::setMax(max);
            }
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_NUMERICNODE_H
