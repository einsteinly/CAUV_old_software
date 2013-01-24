/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CAUV_GUI_MODEL_H__
#define __CAUV_GUI_MODEL_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class NodeItemModel;
class Node;

class NodeUpdateModelNotfication : public QObject {
    Q_OBJECT
    public:
    NodeUpdateModelNotfication(NodeItemModel * model);

    public Q_SLOTS:
        void update();

    Q_SIGNALS:
        void onUpdate(QModelIndex start, QModelIndex end);

    protected:
        NodeItemModel * m_model;
};


class NodeItemModel : public QAbstractItemModel {
    Q_OBJECT
    friend class NodeUpdateModelNotfication;

    public:
        NodeItemModel(boost::shared_ptr<Node> root, QObject * parent = 0);

        QModelIndex indexFromNode(boost::shared_ptr<Node> node) const;
        boost::shared_ptr<Node> nodeFromIndex(QModelIndex const& index) const;

        Qt::ItemFlags flags(const QModelIndex &index) const;
        QVariant data ( const QModelIndex & index, int role = Qt::DisplayRole ) const ;
        QVariant headerData ( int section, Qt::Orientation orientation, int role = Qt::DisplayRole ) const ;

        bool setData ( const QModelIndex & index, const QVariant & value, int role = Qt::EditRole );
        QMimeData * mimeData ( const QModelIndexList & indexes ) const;

        int rowCount ( const QModelIndex & parent = QModelIndex() ) const ;
        int columnCount(const QModelIndex &/*parent*/) const ;

        QModelIndex parent(const QModelIndex &child) const ;
        QModelIndex index(int row, int column, const QModelIndex &parent=QModelIndex()) const ;
        
        boost::shared_ptr<Node> rootNode();

    protected Q_SLOTS:
        void connectUpdater(boost::shared_ptr<Node> node);

    protected:
        boost::shared_ptr<Node> m_root;
        NodeUpdateModelNotfication m_updater;
};

} // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
