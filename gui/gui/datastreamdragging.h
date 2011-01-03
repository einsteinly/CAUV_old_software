#ifndef DATASTREAMDRAGGING_H
#define DATASTREAMDRAGGING_H

#include <QTreeWidgetItem>
#include <QEvent>
#include <QDragEnterEvent>
#include <QDropEvent>

#include <model/auv_model.h>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <common/data_stream.h>

namespace cauv {

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
    void onChange(const T value) {
        std::stringstream stream;
        stream << value;
        this->setText(1, QString::fromStdString(stream.str()));
    }
};

// partial specialization for int8_t as it prints as a char not as an int
// so we cast in to int in the implementation before printing
template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);

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

}

#endif // DATASTREAMDRAGGING_H
