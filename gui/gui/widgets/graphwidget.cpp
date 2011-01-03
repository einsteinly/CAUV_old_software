#include "graphwidget.h"

#include <common/cauv_utils.h>
#include <common/data_stream_tools.h>

#include <boost/bind.hpp>

#include <QMdiSubWindow>
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
    }
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

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> >){
    // TODO: implement a data stream splitter for this
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






GraphArea::GraphArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QMdiArea(parent),
        CauvInterfaceElement(name, auv, node) {
    this->setAcceptDrops(true);
}

void GraphArea::initialise(){
    m_actions->registerCentralView(this, name());
}

void GraphArea::addGraph(GraphWidget *graph){
    QMdiSubWindow * window = addSubWindow(graph);
    window->resize(400, 250);
    window->show();
}

void GraphArea::dropEvent(QDropEvent * event){
    DataStreamDropListener::dropEvent(event);
}

void GraphArea::dragEnterEvent(QDragEnterEvent * event){
    DataStreamDropListener::dragEnterEvent(event);
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream){
    addGraph(new GraphWidget(stream));
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<int> > stream){
    addGraph(new GraphWidget(stream));
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<float> > stream){
    addGraph(new GraphWidget(stream));
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> > stream){
    addGraph(new GraphWidget(stream));
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream){
    addGraph(new GraphWidget(stream));
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream){
    addGraph(new GraphWidget(stream));
}
