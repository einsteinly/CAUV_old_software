#include "graphs.h"

#include <common/cauv_utils.h>
#include <common/data_stream_tools.h>

#include <boost/bind.hpp>

#include <QPen>
#include <QRectF>

#include <qwt_legend.h>

using namespace cauv;


template<class T>
void GraphWidget::addStream(boost::shared_ptr<DataStream<T> > stream){
    // see if this series has already been registered
    if(seriesNames.end() == seriesNames.find(stream->getName())) {
        seriesNames.insert(stream->getName());
        series.insert(boost::make_shared<DataStreamSeriesData<T> >(stream, 1000));
        std::stringstream str;
        str << "Graph " << getName();
        setWindowTitle(QString::fromStdString(str.str()));
    }
}

QSize GraphWidget::sizeHint() const{
    return QSize(400, 250);
}

void GraphWidget::setupPlot() {
    // Insert grid
    m_grid->attach(this);
    QPen pen(Qt::gray, 1, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
    m_grid->setPen(pen);

    this->setCanvasBackground(QColor(Qt::white));
    this->setAutoReplot(true);
    this->setMargin(5);

    // legend
    QwtLegend *legend = new QwtLegend;
    legend->setFrameStyle(QFrame::Box|QFrame::NoFrame);
    this->insertLegend(legend, QwtPlot::BottomLegend);
}

void GraphWidget::dropEvent(QDropEvent * event){
    DataStreamDropListener::dropEvent(event);
}

void GraphWidget::dragEnterEvent(QDragEnterEvent * event){
    DataStreamDropListener::dragEnterEvent(event);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<int> >stream){
    addStream(stream);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<int8_t> >stream){
    addStream(stream);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<float> >stream) {
    addStream(stream);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> >stream) {
    boost::shared_ptr<DataStreamSplitter<floatYPR> > split = boost::make_shared<DataStreamSplitter<floatYPR> >(stream);
    addStream(split->yaw);
    addStream(split->pitch);
    addStream(split->roll);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> >stream){
    addStream(stream);
}


std::string GraphWidget::getName() const{
    std::stringstream title;
    title << "[" << implode(", ", seriesNames) << "]";
    return title.str();
}

