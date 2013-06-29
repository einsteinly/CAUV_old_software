/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
                m_max(100), m_min(0), m_maxSet(false), m_minSet(false), m_wraps(false), m_inverted(false), m_precision(6)
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

            virtual T typedGet() {
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

            virtual void update(QVariant const& value) {
                return NumericNodeBase::update(value.value<T>());
            }

            virtual void typedUpdate(T const& value){
                return NumericNodeBase::update(QVariant::fromValue(value));
            }

            virtual bool set(QVariant const& value) {
                return typedSet(value.value<T>());
            }

            virtual bool typedSet(T const& value){
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

            virtual void typedSetMin(T min){
                NumericNodeBase::setMin(QVariant::fromValue(min));
            }

            virtual void typedSetMax(T max){
                NumericNodeBase::setMax(QVariant::fromValue(max));
            }

            virtual void typedSetNeutral(T neutral){
                NumericNodeBase::setNeutral(QVariant::fromValue(neutral));
            }

            // todo: I think this isn't required now...
            virtual QVariant asNumber(){
                return get();
            }
        };


        template<>
        class NumericNode<BoundedFloat> : public NumericNodeBase {
        public:
            NumericNode(nid_t const& id) : NumericNodeBase(id)
            {
                m_value = QVariant::fromValue(BoundedFloat());
            }

            virtual void typedUpdate(BoundedFloat const& value){
                typedSetMax(value);
                typedSetMin(value);
                typedSetWraps(value);
                NumericNodeBase::update(QVariant::fromValue(value));
            }

            virtual void update(QVariant const& value) {
                typedUpdate(value.value<BoundedFloat>());
            }

            virtual BoundedFloat typedGet() {
                return m_value.value<BoundedFloat>();
            }

            virtual bool typedSet(BoundedFloat const& value){
                return NumericNodeBase::set(QVariant::fromValue(value));
            }

            virtual bool typedSet(float const& value){
                BoundedFloat boundedValue;
                boundedValue.value = value;
                boundedValue.max = getMax().value<float>();
                boundedValue.min = getMin().value<float>();
                return typedSet(boundedValue);
            }

            virtual bool set(QVariant const& value) {
                if(value.userType() != qMetaTypeId<BoundedFloat>()){
                    return typedSet(value.value<float>());
                }
                else return NumericNodeBase::set(value);
            }

            virtual QVariant asNumber() {
                return get().value<BoundedFloat>().value;
            }

            virtual void typedSetMin(BoundedFloat value){
                NumericNodeBase::setMin(value.min);
            }

            virtual void typedSetMax(BoundedFloat value){
                NumericNodeBase::setMax(value.max);
            }

            virtual void typedSetWraps(BoundedFloat value){
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
