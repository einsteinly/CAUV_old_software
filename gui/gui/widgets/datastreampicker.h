#ifndef DATASTREAMPICKER_H
#define DATASTREAMPICKER_H

#include <QDockWidget>
#include <QListView>
#include <QVariant>

#include "ui_datastreampicker.h"
#include "../cauvinterfaceelement.h"
#include "../datastreamdragging.h"

namespace cauv {

    class DataStreamList : public QListView, public DataStreamDragSource {
    public:
        boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > getDataStreams() const;
    };


    template <class T>
    class DataStreamListItem : public QAbstractItemModel {

    public:
        DataStreamListItem(boost::shared_ptr<DataStream<T> > stream, QObject * parent = 0) :
                m_parent(parent), m_stream(stream) {
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
        }

        boost::shared_ptr<DataStream<T> > getDataStream(){
            return m_stream;
        }

        virtual int columnCount(const QModelIndex &) const {
            return 2;
        }

        virtual int rowCount(const QModelIndex &) const {
            return 1;
        }

        virtual Qt::ItemFlags flags(const QModelIndex &index) const{
            return QAbstractListModel::flags(index);
        }

        virtual QVariant data(const QModelIndex &index, int role ) const{
            if(index.column() == 0)
            {
                QVariant var(QString::fromStdString(m_stream->getName()));
                return var;
            } else {
                QVariant var(m_value);
                return var;
            }
        }

        virtual QModelIndex index(int row, int column, const QModelIndex &parent) const {
            if (!parent.isValid())
                return QModelIndex();
            else
                parentItem = static_cast<TreeItem*>(parent.internalPointer());

            TreeItem *childItem = parentItem->child(row);
            if (childItem)
                return createIndex(row, column, childItem);
            else


        }

        virtual QModelIndex parent(const QModelIndex &child) const {
            return m_parent;
        }

    protected:
        boost::shared_ptr<DataStream<T> > m_stream;
        std::string m_value;
        QModelIndex m_parent;
        
        void onChange(const T value) {
            std::stringstream stream;
            stream << value;
            this->setText(1, QString::fromStdString(stream.str()));
        }
    };

    class DataStreamPicker : public QDockWidget, public Ui::DataStreamPicker, public CauvInterfaceElement {
        Q_OBJECT
    public:
        DataStreamPicker(const QString &name, boost::shared_ptr<cauv::AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual void initialise();
    };

}

#endif // DATASTREAMPICKER_H
