#ifndef DATASTREAMPICKER_H
#define DATASTREAMPICKER_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeView>
#include <QTreeWidgetItem>

#include <boost/lexical_cast.hpp>

#include "../utils/treeitem.h"
#include "../cauvinterfaceelement.h"
#include "../datastreamdragging.h"

namespace Ui {
    class DataStreamPicker;
}


namespace cauv {

    /**
      * A DataStreamTreeItemBase is used as the base class for one row in the
      * table of data streams.
      *
      * @author Andy Pritchard
      */
    class DataStreamTreeItemBase : public QTreeWidgetItem {

    public:
        DataStreamTreeItemBase(boost::shared_ptr<DataStreamBase> stream, QTreeWidgetItem * parent);
        virtual ~DataStreamTreeItemBase(){}
        boost::shared_ptr<DataStreamBase> getDataStreamBase();
        virtual bool updateStream(QVariant& ) = 0;

    private:
        boost::shared_ptr<DataStreamBase> m_stream;
    };


    /**
      * A DataStreamTreeItem is responsible for updating the DataStream (if its
      * mutable) and updating the table when the DataStream's value changes.
      * It is also where the name cell is filled in.
      *
      * @author Andy Pritchard
      */
    template<class T>
    class DataStreamTreeItem : public DataStreamTreeItemBase {

    public:
        DataStreamTreeItem(boost::shared_ptr< DataStream<T> > stream, QTreeWidgetItem * parent) :
                DataStreamTreeItemBase(stream, parent), m_stream(stream) {
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
            this->setText(0, QString::fromStdString(stream->getName()));
        }

        DataStreamTreeItem(boost::shared_ptr< DataStream<T> > stream, std::string name, QTreeWidgetItem * parent) :
                DataStreamTreeItemBase(stream, parent), m_stream(stream) {
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
            this->setText(0, QString::fromStdString(name));
        }

        T qVariantToValue(QVariant& value){
            return boost::lexical_cast<T>(value.toString().toStdString());
        }

        virtual bool updateStream(QVariant& value){
            if(!m_stream->isMutable()) return false;

            try {
                boost::shared_ptr<MutableDataStream<T> > stream = boost::shared_static_cast<MutableDataStream<T> >(m_stream);
                stream->set(qVariantToValue(value));
                info() << m_stream->getName() << " value changed to: " << value.toString().toStdString();
                return true;
            } catch (boost::bad_lexical_cast){
                info() << m_stream->getName() << " given a bad value:" << value.toString().toStdString();
                return false;
            }
        }

    protected:
        boost::shared_ptr< DataStream<T> > m_stream;

        void onChange(const T value) {
            std::stringstream stream;
            stream << value;
            this->setText(1, QString::fromStdString(stream.str()));
        }
    };


    // partial specializations
    // need one for int8_t as it prints as a char not as an int, so we cast it
    // to int in the implementation before printing
    template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);
    // another for int8_t for much the same reason but with lexical cast this time
    template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant& value);
    // also need some for out types as lexical cast doesn't knwo what to do
    template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& value);
    template<> sonar_params_t DataStreamTreeItem<sonar_params_t>::qVariantToValue(QVariant& value);
    template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& value);


    /**
      * Interface integration class.
      *
      * @author Andy Pritchard
      */
    class DataStreamPicker : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        DataStreamPicker(const QString &name, boost::shared_ptr<cauv::AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~DataStreamPicker();
        virtual void initialise();

    private:
        Ui::DataStreamPicker *ui;
    };

}


/**
  * The QWidget subclass where the model data is displayed.
  *
  * It's outside the cauv namespace because its used as promoted widget in a ui file
  * TODO: find a way around this
  *
  * @author Andy Pritchard
  */
class DataStreamList : public QTreeWidget, public cauv::DataStreamDragSource {
    Q_OBJECT
public:
    DataStreamList(QWidget * parent) : QTreeWidget(parent){

        // Because QTreeWidgetItem can't let only one cell in a row be editable we have to hack it
        // around a bit. We set the item to be editable when it's double clicked in the editable
        // cell. This is then removed once the focus is lost. It works but its not a very elegant solution
        // TODO: Better would be to use a full QAbstractItemModel implementation but this is quite a
        // big job as QAbstractItemModels are complicated.
        connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), this, SLOT(editStarted(QTreeWidgetItem*,int)));
        connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(editEnded(QTreeWidgetItem*,QTreeWidgetItem*)));
        connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(itemEdited(QTreeWidgetItem*,int)));
    }

    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::DataStreamBase> > > getDataStreams() const;

    private Q_SLOTS:
    void editStarted(QTreeWidgetItem* item, int column){
        if(column == 1){
            cauv::DataStreamTreeItemBase * dsItem = dynamic_cast<cauv::DataStreamTreeItemBase*>(item);
            if(dsItem && dsItem->getDataStreamBase()->isMutable()){
                item->setFlags(item->flags() | Qt::ItemIsEditable);
            } item->setFlags(item->flags() | Qt::ItemIsEditable);
        }
    }

    void itemEdited(QTreeWidgetItem* item, int column){
        if(column == 1){ // 1 is the editable cell that stores the value
            cauv::DataStreamTreeItemBase * dsItem = dynamic_cast<cauv::DataStreamTreeItemBase*>(item);
            if(dsItem && dsItem->getDataStreamBase()->isMutable()){
                QVariant v = item->data(column, Qt::DisplayRole);
                if(!v.toString().isEmpty()) {
                    // if the item is marked as editable then the change came from a user interaction
                    // not a stream update
                    // TODO: find a better way of doing this. it's a bit hacky.
                    if(item->flags() & Qt::ItemIsEditable) {
                        if(dsItem->updateStream(v)) item->setBackground(1, QBrush());
                        else  {
                            QBrush b(Qt::DiagCrossPattern);
                            b.setColor(QColor::fromRgb(224, 128, 128));
                            item->setBackground(1, b);
                        }
                    }
                }
            }
        }
    }

    void editEnded(QTreeWidgetItem* , QTreeWidgetItem* previous){
        if(previous) // this method resets the editable flag if the focus is lost
            previous->setFlags(previous->flags() & (~Qt::ItemIsEditable));
    }
};

#endif // DATASTREAMPICKER_H
