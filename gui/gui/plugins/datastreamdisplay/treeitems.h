#ifndef TREEITEMS_H
#define TREEITEMS_H

#include <QVariant>
#include <QTreeWidgetItem>
#include <cv.h>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/compoundnodes.h>
#include <gui/core/model/nodes/stringnode.h>
#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/nodes/imagenode.h>




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
            virtual bool updateNode(QVariant&);

        public Q_SLOTS:
            NodeTreeItemBase * addNode(boost::shared_ptr<NodeBase> node);
            void updateIcon(int cell, QImage &image);
            void updateIcon(int cell, const Image &image);
            void updateValue(const QString value);
            bool filter(QString value);
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

            virtual T qVariantToValue(QVariant& ) {
                // should overide this method
                throw boost::bad_lexical_cast();
            }

            virtual bool updateNode(QVariant& value){
                if(!m_node->isMutable()) return false;

                try {
                    m_node->set(qVariantToValue(value));
                    info() << m_node->nodePath() << " value changed to: " << value.toString().toStdString();
                    return true;
                } catch (boost::bad_lexical_cast){
                    info() << m_node->nodePath() << " given a bad value:" << value.toString().toStdString();
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

                return m_node->numericVariantFromString(value);
            }

        protected Q_SLOTS:
            void onChange(numeric_variant_t value){
                std::stringstream str;
                str.flags(str.flags() | std::stringstream::fixed);
                str.precision(m_node->getPrecision());
                str << value << " " << m_node->getUnits();
                updateValue(QString::fromStdString(str.str()));
            }

        protected:
            boost::shared_ptr<NumericNode> m_node;
        };





        class GroupingNodeTreeItem : public NodeTreeItem<std::string> {
            Q_OBJECT
        public:
            GroupingNodeTreeItem(boost::shared_ptr<GroupingNode> node, QTreeWidgetItem * parent) :
                    NodeTreeItem<std::string>(node, parent) {
                node->connect(node.get(), SIGNAL(onUpdate(std::string)), this, SLOT(onChange(std::string)));
            }

        protected Q_SLOTS:
            void onChange(std::string value){
                setText(0, QString::fromStdString(value));
            }
        };




        class ImageNodeTreeItem : public NodeTreeItem<image_variant_t> {
            Q_OBJECT
        public:
            ImageNodeTreeItem(boost::shared_ptr<ImageNode> node, QTreeWidgetItem * parent) :
                    NodeTreeItem<image_variant_t>(node, parent) {
                node->connect(node.get(), SIGNAL(onUpdate(image_variant_t)), this, SLOT(updateIcon(image_variant_t)));
                onChange(node->get());
            }


        protected Q_SLOTS:

            void updateIcon(int cell, QImage &image){
                this->setIcon(cell, QIcon(QPixmap::fromImage(image)));
            }

            void updateIcon(image_variant_t image){
                try {
                    cv::Mat mat_rgb;
                    cv::cvtColor(image->mat(), mat_rgb, CV_BGR2RGB);

                    QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                                           mat_rgb.rows, QImage::Format_RGB888);

                    this->updateIcon(1, qImage);

                } catch (...){
                    error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__;
                }
            }

        };




        class StringNodeTreeItem : public NodeTreeItem<std::string> {
            Q_OBJECT
        public:
            StringNodeTreeItem(boost::shared_ptr<StringNode> node, QTreeWidgetItem * parent) :
                    NodeTreeItem<std::string>(node, parent) {
                node->connect(node.get(), SIGNAL(onUpdate(std::string)), this, SLOT(onChange(std::string)));
                onChange(node->get());
            }

        public Q_SLOTS:
            void onChange(std::string str){
                updateValue(QString::fromStdString(str));
            }
        };


        class FloatYPRNodeTreeItem : public NodeTreeItem<floatYPR> {
            Q_OBJECT
        public:
            FloatYPRNodeTreeItem(boost::shared_ptr<FloatYPRNode> node, QTreeWidgetItem * parent) :
                    NodeTreeItem<floatYPR>(node, parent) {
            }
        };

    } // namespace gui
} // namespace cauv

#endif // TREEITEMS_H
