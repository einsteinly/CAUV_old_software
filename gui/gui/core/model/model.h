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

#include "nodes/vehiclenode.h"
#include "nodes/numericnode.h"

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
                info() << "notifying model of update";
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
            void createUpdater(boost::shared_ptr<Node> node);

        protected:
            boost::shared_ptr<Node> m_root;
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
