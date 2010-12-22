#ifndef DATASTREAMTREEITEM_H
#define DATASTREAMTREEITEM_H

#include <QTreeWidgetItem>
#include <QEvent>
#include <QDragEnterEvent>
#include <QDropEvent>

#include <model/auv_model.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <common/data_stream.h>

template <class T>
class DataStreamTreeItem : public QTreeWidgetItem {

public:
    DataStreamTreeItem(QTreeWidgetItem * parent, boost::shared_ptr<DataStream<T> > stream) :
            QTreeWidgetItem(parent),m_stream(stream) {
        stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
        setText(0, QString::fromStdString(stream->getName()));
    }

    boost::shared_ptr<DataStream<T> > getDataStream(){
        return m_stream;
    }

protected:
    boost::shared_ptr<DataStream<T> > m_stream;

    void onChange(T value){
        std::stringstream stream;
        stream << value;
        this->setText(1, QString::fromStdString(stream.str()));
    }
};



class DataStreamDropListener {
public:
    virtual void dragEnterEvent(QDragEnterEvent *event);
    virtual void dropEvent(QDropEvent *event);

protected:
    virtual void routeStream(QTreeWidgetItem * s);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> > stream);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
    virtual void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
};


#endif // DATASTREAMTREEITEM_H
