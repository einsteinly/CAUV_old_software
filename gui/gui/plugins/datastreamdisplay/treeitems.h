#ifndef TREEITEMS_H
#define TREEITEMS_H

#include <QVariant>
#include <QTreeWidgetItem>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <gui/core/model/nodes.h>
#include <gui/core/model/model.h>

namespace cauv {
    namespace gui {

        /**
  * A NodeTreeItemBase is used as the base class for one row in the
  * table of data streams.
  *
  * @author Andy Pritchard
  */
        class NodeTreeItemBase : public QObject, public QTreeWidgetItem {
            Q_OBJECT
        public:
            NodeTreeItemBase(boost::shared_ptr<NodeBase> node, QTreeWidgetItem * parent);
            virtual ~NodeTreeItemBase(){}
            boost::shared_ptr<NodeBase> getNode();
            virtual bool updateNode(QVariant& value) = 0;

        protected Q_SLOTS:
            void updateIcon(int cell, QImage &image);
            void updateIcon(int cell, const Image &image);
            void updateValue(const QString value);

        private:
            boost::shared_ptr<NodeBase> m_node;
        };


        /**
  * A NodeTreeItem is responsible for updating the node (if its
  * mutable) and updating the table when the nodes value changes.
  * It is also where the name cell is filled in.
  *
  * @author Andy Pritchard
  */

        template<class T>
        class NodeTreeItem : public NodeTreeItemBase {

        public:
            NodeTreeItem(boost::shared_ptr<Node<T> > node, QTreeWidgetItem * parent) :
                    NodeTreeItemBase(node, parent), m_node(node) {
            }

            virtual T qVariantToValue(QVariant& v) = 0;

            virtual bool updateNode(QVariant& value){
                if(!m_node->isMutable()) return false;

                try {
                    m_node->set(qVariantToValue(value));
                    info() << m_node->nodeName() << " value changed to: " << value.toString().toStdString();
                    return true;
                } catch (boost::bad_lexical_cast){
                    info() << m_node->nodeName() << " given a bad value:" << value.toString().toStdString();
                    return false;
                }
            }

            virtual void onChange(T value){
                std::stringstream str;
                str << value;
                updateValue(QString::fromStdString(str.str()));
            }

        protected:
            boost::shared_ptr<Node<T> > m_node;
        };




        class NumericNodeTreeItem : public NodeTreeItem<numeric_variant_t> {
            Q_OBJECT
        public:
            NumericNodeTreeItem(boost::shared_ptr<NumericNode> node, QTreeWidgetItem * parent) :
                    NodeTreeItem<numeric_variant_t>(node, parent), m_node(node) {
                node->connect(node.get(), SIGNAL(onUpdate(numeric_variant_t)), this, SLOT(onChange(numeric_variant_t)));
                onChange(node->get());
            }
            
            virtual numeric_variant_t qVariantToValue(QVariant& v){
                std::string value = v.toString().toStdString();

                try {
                    int unitsLength = m_node->getUnits().length();
                    // take out where the units should be
                    std::string units = value.substr(value.length()-unitsLength, unitsLength);
                    if(units == m_node->getUnits()){
                        value = value.substr(0, value.length()-unitsLength);
                    }
                } catch (std::out_of_range){ }

                // TODO: correct parsing of type, either int or float should so
                return numeric_variant_t(boost::lexical_cast<float>(value));
            }

        protected Q_SLOTS:
            void onChange(numeric_variant_t value){
                std::stringstream str;
                str << value << " " << m_node->getUnits();
                updateValue(QString::fromStdString(str.str()));
            }

        protected:
            boost::shared_ptr<NumericNode> m_node;
        };

        // partial specializations
        // need one for int8_t as it prints as a char not as an int, so we cast it
        // to int in the implementation before printing
        /*template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);
    template<> void DataStreamTreeItem<Image>::onChange(const Image value);
    // another for int8_t for much the same reason but with lexical cast this time
        template<> int8_t NodeTreeItem<int8_t>::qVariantToValue(QVariant& value);
        // also need some for out types as lexical cast doesn't know what to do
        template<> MotorDemand NodeTreeItem<MotorDemand>::qVariantToValue(QVariant& value);
        template<> Image NodeTreeItem<Image>::qVariantToValue(QVariant& value);
*/
    } // namespace gui
} // namespace cauv

#endif // TREEITEMS_H
