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

#include <gui/core/model/node.h>
#include <gui/core/model/paramvalues.h>

#include <QMetaType>
#include <QVariant>

namespace cauv {
    namespace gui {

        class NumericNodeBase : public Node {
            Q_OBJECT

        protected:
            NumericNodeBase(nid_t const& id) : Node(id, nodeType<NumericNodeBase>()),
                m_max(100), m_min(0), m_maxSet(false), m_minSet(false), m_wraps(false), m_inverted(false), m_precision(3)
            {
            }


        public:

            virtual std::string getUnits() const {
                return m_units;
            }

            virtual void setUnits(std::string const& units) {
                m_units = units;
            }

            virtual bool isMaxSet() const {
                return m_maxSet;
            }

            virtual bool isMinSet() const{
                return m_minSet;
            }

            virtual bool getWraps() const {
                return m_wraps;
            }

            virtual bool isInverted() const{
                return m_inverted;
            }

            virtual void setInverted(bool inverted){
                m_inverted = inverted;

                Q_EMIT paramsUpdated();
            }

            virtual void setWraps(bool wraps){
                m_wraps = wraps;

                Q_EMIT paramsUpdated();
            }

            virtual void setPrecision(unsigned int precision){
                m_precision = precision;

                Q_EMIT paramsUpdated();
            }

            virtual unsigned int getPrecision() const {
                return m_precision;
            }

            virtual void setMin(QVariant const& min){
                m_minSet = true;

                // only update if its actually changed
                if (min == getMin()) return;
                m_min = min;

                Q_EMIT paramsUpdated();
            }

            virtual void setMax(QVariant const& max){
                m_maxSet = true;

                // only update if its actually changed
                if (max == getMax()) return;

                m_max = max;

                Q_EMIT paramsUpdated();
            }

            virtual void setNeutral(QVariant const& neutral){
                // only update if its actually changed
                if (neutral == getNeutral()) return;

                m_neutral = neutral;

                Q_EMIT paramsUpdated();
            }

            virtual QVariant getMax() const {
                return m_max;
            }

            virtual QVariant getMin() const {
                return m_min;
            }

            virtual QVariant getNeutral() const {
                return m_neutral;
            }

            virtual QVariant asNumber() = 0;

        Q_SIGNALS:
            void paramsUpdated();

        protected:
            std::string m_units;
            QVariant m_max;
            QVariant m_min;
            bool m_maxSet, m_minSet, m_wraps, m_inverted;
            unsigned int m_precision;
            QVariant m_neutral;
        };


        template<class T>
        class NumericNode : public NumericNodeBase {
        public:
            NumericNode(nid_t const& id) : NumericNodeBase(id)
            {
                m_value = QVariant::fromValue(T());
            }

            virtual T get() {
                return m_value.value<T>();
            }

            virtual T wrap(T const& value) const {
                // work out the range

                QVariant minV = getMin();
                QVariant maxV = getMax();

                T min = minV.value<T>();
                T max = maxV.value<T>();
                T range = max - min;

                if(value < min){
                    T var = value + range;
                    return wrap(var);
                } else if (value > max){
                    T var = value - range;
                    return wrap(var);
                } else return value;
                return value;
            }

            virtual bool set(QVariant const& value) {
                return set(value.value<T>());
            }

            virtual bool set(T const& value){
                debug() << "NumericNode::set() cleaning value";

                QVariant minV = getMin();
                QVariant maxV = getMax();

                // validate the input as this has come from
                // a user.
                T cleanVal = value;

                // first check if the data wraps
                if (getWraps())
                    cleanVal = wrap(value);
                // if not then maybe it's clamped?
                else {
                    // upper bound check
                    if(isMaxSet() && cleanVal > maxV.value<T>())
                        cleanVal = maxV.value<T>();

                    // lower bound check
                    if(isMinSet() && cleanVal < minV.value<T>())
                        cleanVal = minV.value<T>();
                }

                return NumericNodeBase::set(QVariant::fromValue(cleanVal));
            }

            virtual void setMin(T min){
                NumericNodeBase::setMin(QVariant::fromValue(min));
            }

            virtual void setMax(T max){
                NumericNodeBase::setMax(QVariant::fromValue(max));
            }

            virtual void setNeutral(T neutral){
                NumericNodeBase::setNeutral(QVariant::fromValue(neutral));
            }

            virtual QVariant asNumber(){
                return get();
            }

        private:
            void setMin(QVariant const&){}
            void setMax(QVariant const&){}
            void setNeutral(QVariant const&){}
        };


        template<>
        class NumericNode<BoundedFloat> : public NumericNodeBase {
        public:
            NumericNode(nid_t const& id) : NumericNodeBase(id)
            {
                m_value = QVariant::fromValue(BoundedFloat());
            }

            virtual void update(BoundedFloat const& value){
                info() << "update(BoundedFloat) - " << value.value;
                setMax(value);
                setMin(value);
                setWraps(value);
                NumericNodeBase::update(QVariant::fromValue(value));
            }

            virtual void update(QVariant const& value) {
                info() << "update(QVariant)";
                update(value.value<BoundedFloat>());
            }

            virtual bool set(BoundedFloat const& value){
                info() << "set(BoundedFloat) - " << value.value;
                return set(QVariant::fromValue(value));
            }

            virtual bool set(float const& value){
                info() << "set(double)";
                BoundedFloat boundedValue;
                boundedValue.value = value;
                boundedValue.max = getMax().value<float>();
                boundedValue.min = getMin().value<float>();
                return set(boundedValue);
            }

            virtual bool set(QVariant const& value) {
                info() << "set(QVariant)";
                if(value.userType() != qMetaTypeId<BoundedFloat>()){
                    return set(value.value<float>());
                }
                else return NumericNodeBase::set(value);
            }

            virtual QVariant asNumber() {
                return get().value<BoundedFloat>().value;
            }

        private:
            virtual void setMin(BoundedFloat value){
                info() << "setMin(BoundedFloat)" << value.min;
                NumericNodeBase::setMin(value.min);
            }

            virtual void setMax(BoundedFloat value){
                info() << "setMax(BoundedFloat)" << value.max;
                NumericNodeBase::setMax(value.max);
            }

            virtual void setWraps(BoundedFloat value){
                info() << "setWraps(BoundedFloat)" << value.type;
                NumericNodeBase::setWraps(value.type==BoundedFloatType::Wraps);
            }
        };


        class BooleanNode : public NumericNode<bool> {
        public:
            BooleanNode(const nid_t id) : NumericNode<bool>(id) {
                // change the type of boolean nodes so they're not really
                // numeric nodes, but we get all the typing stuff above for free
                type = nodeType<BooleanNode>();
            }
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_NUMERICNODE_H
