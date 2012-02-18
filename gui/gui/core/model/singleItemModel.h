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

#ifndef __CAUV_GUI_F_MODEL_H__
#define __CAUV_GUI_F_MODEL_H__

#include <QAbstractItemModel>
#include <QModelIndex>

#include <boost/shared_ptr.hpp>

#include <gui/core/model/node.h>

namespace cauv{
namespace gui{

/* Single-Node model, which can be used with TreeViews */
class SingleNodeItemModel: public QAbstractItemModel{
    Q_OBJECT

    public:
        SingleNodeItemModel(boost::shared_ptr<Node> node, QObject *parent = 0)
            : QAbstractItemModel(parent), m_node(node){
        }

        QModelIndex index(int row, int col, const QModelIndex & parent = QModelIndex()) const{
            Q_UNUSED(parent);
            if(row != 0 || col != 0)
                return QModelIndex();
            return createIndex(0,0,m_node.get());
        }

        QModelIndex parent(const QModelIndex &child) const{
            Q_UNUSED(child);
            return QModelIndex();
        }

        int rowCount(const QModelIndex &parent = QModelIndex()) const{
            // Only top level exists:
            if(parent.isValid())
                return 0;
            else
                return 1;
        }

        int columnCount(const QModelIndex &parent = QModelIndex()) const{
            Q_UNUSED(parent);
            return 1;
        }

        QVariant data(const QModelIndex &index, int role) const{
            if(!index.isValid() || !m_node)
                return QVariant();

            switch (role){
                case Qt::EditRole:
                case Qt::DisplayRole:
                    return m_node->get();
                default:
                    return QVariant();
            }
        }

        QVariant headerData(int section, Qt::Orientation orientation, int role) const{
            if(role != Qt::DisplayRole)
                return QVariant();

            if(orientation == Qt::Horizontal)
                return QString("Column %1").arg(section);
            else
                return QString("Row %1").arg(section);
        }

        Qt::ItemFlags flags(const QModelIndex &index) const{
            if(!index.isValid())
                return Qt::NoItemFlags;
            // do not allow selection
            return Qt::ItemIsEnabled | Qt::ItemIsEditable;
        }

        bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole){
            if(index.isValid() && role == Qt::EditRole && index.row() == 0 && index.column() == 0){
                m_node->set(value);
                Q_EMIT dataChanged(index, index);
                return true;
            }
            return false;
        }

    private:
        boost::shared_ptr<Node> m_node;
};


} // namspace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_MODEL_H__

