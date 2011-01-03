
#include "datastreamdragging.h"

using namespace cauv;


template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    this->setText(1, QString::fromStdString(stream.str()));
}


void DataStreamDropListener::dragEnterEvent(QDragEnterEvent *event)
{
    event->acceptProposedAction();
}

void DataStreamDropListener::dropEvent(QDropEvent *event)
{
    event->acceptProposedAction();

    QTreeWidget * tree = dynamic_cast<QTreeWidget*> (event->source());
    if(!tree) // drop didn't come from the tree widget
        return;

    // if we've recieved a drop event from a QTreeWidget
    // we need to route the selected items, assuming they
    // are DataStreamTreeItem's
    QList<QTreeWidgetItem*> items = tree->selectedItems();
    QList<QTreeWidgetItem*>::iterator i;
    for (i = items.begin(); i != items.end(); ++i)
        routeStream(*i);
}

void DataStreamDropListener::routeStream(QTreeWidgetItem * s){
    if(dynamic_cast<DataStreamTreeItem<int8_t>*>(s))
        onStreamDropped(((DataStreamTreeItem<int8_t>*)s)->getDataStream());

    if(dynamic_cast<DataStreamTreeItem<int>*>(s))
        onStreamDropped(((DataStreamTreeItem<int>*)s)->getDataStream());

    if(dynamic_cast<DataStreamTreeItem<float>*>(s))
        onStreamDropped(((DataStreamTreeItem<float>*)s)->getDataStream());

    if(dynamic_cast<DataStreamTreeItem<uint16_t>*>(s))
        onStreamDropped(((DataStreamTreeItem<uint16_t>*)s)->getDataStream());

    if(dynamic_cast<DataStreamTreeItem<autopilot_params_t>*>(s))
        onStreamDropped(((DataStreamTreeItem<autopilot_params_t>*)s)->getDataStream());

    if(dynamic_cast<DataStreamTreeItem<floatYPR>*>(s))
        onStreamDropped(((DataStreamTreeItem<floatYPR>*)s)->getDataStream());
}


void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<int8_t> > ){

}

void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<int> > ){

}

void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<float> > ){

}

void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> > ){

}

void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > ){

}

void DataStreamDropListener::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > ){

}
