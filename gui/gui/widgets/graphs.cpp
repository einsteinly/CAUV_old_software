#include "graphs.h"

#include <common/cauv_utils.h>
#include <common/data_stream_tools.h>

#include <boost/bind.hpp>

#include <QPen>
#include <QRectF>

#include <qwt_painter.h>
#include <qwt_legend.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_canvas.h>
#include <qwt_curve_fitter.h>
#include <qwt_scale_widget.h>

using namespace cauv;


template<class T>
void GraphWidget::addStream(boost::shared_ptr<DataStream<T> > stream){
    // see if this series has already been registered
    if(m_seriesNames.end() == m_seriesNames.find(stream->getName())) {
        // series data
        DataStreamSeriesData<T> * series = new DataStreamSeriesData<T>(stream, 1000);
        m_seriesNames.insert(stream->getName());

        // series plotter
        std::stringstream str;
        str << stream->getName();
        if(stream->getUnits().length() > 0)
            str << " (" <<  stream->getUnits() << ")";
        QwtPlotCurve * curve = new QwtPlotCurve(QString::fromStdString(str.str()));
        curve->setCurveFitter(new QwtSplineCurveFitter());
        curve->setData(series);
        curve->attach(this);

        curve->setPaintAttribute(QwtPlotCurve::ClipPolygons, true);
        curve->setRenderHint(QwtPlotCurve::RenderAntialiased,true);



        // set window title
        std::stringstream name;
        name << "Graph " << getName();
        setWindowTitle(QString::fromStdString(name.str()));
    }
}

QSize GraphWidget::sizeHint() const{
    return QSize(400, 250);
}

void GraphWidget::setupPlot() {
    // Insert grid
    m_grid->attach(this);
    QPen pen(Qt::gray, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin);
    m_grid->setPen(pen);

    this->setCanvasBackground(QColor(Qt::white));

    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setStyleSheet("QwtPlotCanvas {border: 1px dotted gray}");
    canvas()->setLineWidth(1);
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
    canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, true);
    canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, true);

    QwtPainter::setPolylineSplitting(true);

    setContentsMargins(0, 5, 5, 0);

    setAxisFont(yLeft, QFont("Arial", 7, -1, false));
    setAxisFont(xBottom, QFont("Arial", 7, -1, false));
    setAxisTitle(xBottom, "Time (s)");

    QPalette palette;
    palette.setColor(QPalette::WindowText, Qt::gray);
    palette.setColor(QPalette::Foreground, Qt::gray);
    axisWidget(xBottom)->setPalette(palette);
    axisWidget(yLeft)->setPalette(palette);

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
    title << "[" << implode(", ", m_seriesNames) << "]";
    return title.str();
}

