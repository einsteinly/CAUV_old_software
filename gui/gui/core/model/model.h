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

#ifndef GUI_MODEL_H
#define GUI_MODEL_H

#include <QAbstractItemModel>
#include <QMimeData>
#include <QUrl>

#include "node.h"

#include <generated/types/message.h>

#include "../model/nodes/vehiclenode.h"


namespace cauv {
    namespace gui {

        class MessageGenerator;


        // !!! todo: this class shouldn't exist. Vehicle config should be
        // sent by the vehicle when the GUI connects and not hardcoded
        // into the gui.
        class RedHerring : public Vehicle
        {
            Q_OBJECT

        public:
            friend class VehicleRegistry;

        protected:
            RedHerring(std::string name);
            virtual void initialise();

        protected Q_SLOTS:
            void setupMotor(boost::shared_ptr<Node>);
            void setupAutopilot(boost::shared_ptr<Node> node);
        };


        class ModelIndexUpdateNotfication : public QObject {
            Q_OBJECT
        public:
            ModelIndexUpdateNotfication(QModelIndex start, QModelIndex end) :
                m_start(start), m_end(end) {

                debug(4) << "ModelIndexUpdateNotification()";

            }

        public Q_SLOTS:
            void update(){
                Q_EMIT onUpdate(m_start, m_end);
            }

            Q_SIGNALS:
            void onUpdate(QModelIndex start, QModelIndex end);

        protected:
            QModelIndex m_start, m_end;
        };


        class NodeItemModel : public QAbstractItemModel {
            Q_OBJECT

        public:
            NodeItemModel(boost::shared_ptr<Node> root, QObject * parent = 0) :
                QAbstractItemModel(parent), m_root(root){
                connect(root.get(), SIGNAL(structureChanged()), this, SIGNAL(layoutChanged()));
                createUpdater(root);
            }

            Qt::ItemFlags flags(const QModelIndex &index) const{
                void * ptr = index.internalPointer();
                Node * node = static_cast<Node *>(ptr);
                if(index.column()!=0 && node->isMutable()) // column 0 is the node name which can't be chaged
                    return QAbstractItemModel::flags(index) | Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
                else return QAbstractItemModel::flags(index) | Qt::ItemIsDragEnabled;
            }

            QVariant data ( const QModelIndex & index, int role = Qt::DisplayRole ) const {

                void * ptr = index.internalPointer();
                Node * node = static_cast<Node *>(ptr);

                switch (role){
                case Qt::EditRole:
                case Qt::DisplayRole:
                    if (index.column() == 0)
                        return QVariant(QString::fromStdString(node->nodeName()));
                    else return node->get();
                break;
                case Qt::DecorationRole:
                break;
                case Qt::ToolTipRole:
                    return QVariant(QString::fromStdString(node->nodePath()));
                break;
                case Qt::StatusTipRole:
                break;
                case Qt::WhatsThisRole:
                break;
                case Qt::SizeHintRole:
                break;
                case Qt::FontRole:
                break;
                case Qt::TextAlignmentRole:
                break;
                case Qt::BackgroundRole:
                break;
                case Qt::ForegroundRole:
                break;
                case Qt::CheckStateRole:
                break;
                case Qt::AccessibleTextRole:
                break;
                case Qt::AccessibleDescriptionRole:
                break;
                default: break;
                }
                return QVariant();
            }

            QVariant headerData ( int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const {

                Q_UNUSED(orientation);

                switch (role){
                case Qt::ToolTipRole:
                case Qt::EditRole:
                case Qt::DisplayRole:
                    if (orientation == Qt::Horizontal && section == 0) return "Name";
                    if (orientation == Qt::Horizontal && section == 1) return "Value";
                    else return QString("%1").arg(section);
                break;
                case Qt::DecorationRole:
                break;
                case Qt::StatusTipRole:
                break;
                case Qt::WhatsThisRole:
                break;
                case Qt::SizeHintRole:
                break;
                case Qt::FontRole:
                break;
                case Qt::TextAlignmentRole:
                break;
                case Qt::BackgroundRole:
                break;
                case Qt::ForegroundRole:
                break;
                case Qt::CheckStateRole:
                break;
                case Qt::AccessibleTextRole:
                break;
                case Qt::AccessibleDescriptionRole:
                break;
                default: break;
                }
                return QVariant();
            }

            bool setData ( const QModelIndex & index, const QVariant & value, int role = Qt::EditRole ) {

                void * ptr = index.internalPointer();
                Node * node = static_cast<Node *>(ptr);

                switch (role){
                case Qt::DisplayRole:
                    // can't be edited
                break;
                case Qt::EditRole:
                    if (node->set(value))
                    {
                        Q_EMIT dataChanged(index, index);
                        return true;
                    }
                break;
                default: break;
                }
                return false;
            }

            QMimeData * mimeData ( const QModelIndexList & indexes ) const {

                QMimeData *mimeData = new QMimeData;
                QList<QUrl> urls;

                foreach (QModelIndex const & index, indexes){
                    if(index.column() == 0) {
                        // get path from the node
                        Node * node = static_cast<Node *>(index.internalPointer());
                        QUrl url = QUrl();
                        url.setScheme("varstream");
                        boost::shared_ptr<Vehicle> vehicleNode = node->getClosestParentOfType<Vehicle>();
                        url.setHost(QString::fromStdString(vehicleNode->nodeName()));
                        url.setPath(QString::fromStdString(node->nodePath()).remove(url.host().prepend("/")));
                        urls.append(url);
                        debug(5) << url.toString().toStdString() << "added to drag";
                    }
                }

                mimeData->setUrls(urls);
                return mimeData;
            }

            int rowCount ( const QModelIndex & parent = QModelIndex() ) const {
                Node *parentItem;
                if (!parent.isValid())
                    parentItem = m_root.get();
                else
                    parentItem = static_cast<Node*>(parent.internalPointer());

                return parentItem->getChildren().size();
            }

            int columnCount(const QModelIndex &/*parent*/) const {
                // !!! todo: variable column sizes
                return 2;
            }

            QModelIndex parent(const QModelIndex &child) const {
                if (!child.isValid())
                    return QModelIndex();

                Node * childNode = static_cast<Node *>(child.internalPointer());

                try {
                    // work out the row of the parent with respect to its siblings
                    Node * parent = childNode->getParent().get();
                    Node * parentsParent;
                    try {
                        parentsParent = parent->getParent().get();
                    } catch (std::out_of_range){
                        parentsParent = m_root.get();
                    }

                    int row = 0;
                    foreach (boost::shared_ptr<Node> const & c, parentsParent->getChildren()){
                        if (c.get() == parent) break;
                        row++;
                    }
                    // column is 0 because other columns don't ever have children in this
                    // model
                    return createIndex(row, 0, parent);

                } catch (std::out_of_range ex){
                        return QModelIndex();
                }
            }

            QModelIndex index(int row, int column, const QModelIndex &parent) const {
                if (!hasIndex(row, column, parent))
                    return QModelIndex();

                Node * parentNode;

                if(!parent.isValid())
                    parentNode = m_root.get();
                else parentNode = static_cast<Node *>(parent.internalPointer());

                try {
                    Node * childNode = parentNode->getChildren().at(row).get();
                    return createIndex(row, column, childNode);
                } catch (std::out_of_range){
                    return QModelIndex();
                }
            }

        protected Q_SLOTS:
            void createUpdater(boost::shared_ptr<Node> node){

                int row = 0;
                try {
                    foreach (boost::shared_ptr<Node> const& child, node->getParent()->getChildren()){
                        if (child.get() == node.get()) break;
                        row++;
                    }
                } catch (std::out_of_range) {}

                QModelIndex index = createIndex(row, 1, node.get());

                // !!! todo: fix memory leak
                ModelIndexUpdateNotfication * updater = new ModelIndexUpdateNotfication(index, index);
                node->connect(node.get(), SIGNAL(onUpdate()), updater, SLOT(update()));
                this->connect(updater, SIGNAL(onUpdate(QModelIndex,QModelIndex)), this, SIGNAL(dataChanged(QModelIndex, QModelIndex)));

                foreach (boost::shared_ptr<Node> const & child, node->getChildren()){
                    createUpdater(child);
                }

                connect(node.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(createUpdater(boost::shared_ptr<Node>)));
            }


        protected:
            boost::shared_ptr<Node> m_root;
        };


    } // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
