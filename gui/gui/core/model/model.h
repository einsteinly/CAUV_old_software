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

#ifndef __CAUV_GUI_MODEL_H__
#define __CAUV_GUI_MODEL_H__

#include <QAbstractItemModel>
#include <QMimeData>
#include <QModelIndex>

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/vehiclenode.h>
#include <gui/core/model/nodes/numericnode.h>

#include <generated/types/message.h>

namespace cauv {
namespace gui {


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

class NodeItemModel;

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
        QModelIndex index(int row, int column, const QModelIndex &parent) const ;

    protected Q_SLOTS:
        void connectUpdater(boost::shared_ptr<Node> node);

    protected:
        boost::shared_ptr<Node> m_root;
        NodeUpdateModelNotfication m_updater;
};

} // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
