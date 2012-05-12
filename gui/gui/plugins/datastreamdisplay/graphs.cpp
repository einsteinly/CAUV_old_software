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

#include "graphs.h"
#include "datastreamdisplay/ui_graphs.h"

#include <utility/data_stream_tools.h>
#include <utility/string.h>

#include <boost/bind.hpp>

#include <QPen>
#include <QRectF>

#include <qwt_painter.h>
#include <qwt_legend.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_canvas.h>
#include <qwt_curve_fitter.h>
#include <qwt_scale_widget.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_magnifier.h>

using namespace cauv;


const QColor GraphWidget::colours[] = {
    Qt::darkBlue, Qt::red, Qt::darkGreen,  Qt::yellow,
    Qt::darkCyan, Qt::darkGray, Qt::darkMagenta, Qt::darkRed, Qt::darkYellow,
    Qt::blue, Qt::cyan, Qt::gray, Qt::green, Qt::magenta,
};


template<class T>
void GraphWidget::addStream(boost::shared_ptr<DataStream<T> > stream){
    // see if this series has already been registered
    if(m_seriesNames.end() == m_seriesNames.find(stream->getName())) {
        // series data
        DataStreamSeriesData<T> * series = new DataStreamSeriesData<T>(stream, 1000);
        m_seriesNames.insert(stream->getName());

        m_recorderView->addRecorder(series);

        // series plotter
        std::stringstream str;
        str << stream->getName();
        if(stream->getUnits().length() > 0)
            str << " (" <<  stream->getUnits() << ")";
        QwtPlotCurve * curve = new QwtPlotCurve(QString::fromStdString(str.str()));
        curve->setCurveFitter(new QwtSplineCurveFitter());
        curve->setData(series);
        curve->attach(m_plot);

        curve->setPaintAttribute(QwtPlotCurve::ClipPolygons, true);
        curve->setRenderHint(QwtPlotCurve::RenderAntialiased,true);

        curve->setPen(QPen(GraphWidget::colours[(m_seriesNames.size()-1)%14]));

        // set window title
        setWindowTitle(QString::fromStdString(getName()));
    }
}

GraphWidget::~GraphWidget(){
    delete ui;
}

QSize GraphWidget::sizeHint() const{
    return QSize(400, 250);
}

void GraphWidget::setupPlot() {

    ui->widgets->addWidget(m_plot);
    QwtPainter::setPolylineSplitting(true);

    // Insert grid
    QwtPlotGrid * grid = new QwtPlotGrid();
    grid->attach(m_plot);
    QPen pen(Qt::gray, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin);
    grid->setPen(pen);


    // canvas
    m_plot->canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    m_plot->canvas()->setStyleSheet("QwtPlotCanvas {border: 1px dotted gray}");
    m_plot->canvas()->setLineWidth(1);
    m_plot->canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
    // TODO: This api was changed in between 6.0.0-rc5 and 6.0.0-svn-r922
    //m_plot->canvas()->setPaintAttribute(QwtPlotCanvas::BackingStore, true);
    //m_plot->canvas()->setPaintAttribute(QwtPlotCanvas::Opaque, true);

    // plot
    m_plot->setCanvasBackground(QColor(Qt::white));
    m_plot->setContentsMargins(0, 5, 5, 0);

    // axes
    m_plot->setAxisFont(QwtPlot::yLeft, QFont("Arial", 7, -1, false));
    m_plot->setAxisFont(QwtPlot::xBottom, QFont("Arial", 7, -1, false));
    m_plot->setAxisTitle(QwtPlot::xBottom, "Time (s)");

    QPalette palette;
    palette.setColor(QPalette::WindowText, Qt::gray);
    palette.setColor(QPalette::Foreground, Qt::gray);
    m_plot->axisWidget(QwtPlot::xBottom)->setPalette(palette);
    m_plot->axisWidget(QwtPlot::yLeft)->setPalette(palette);

    QwtText title = m_plot->axisWidget(QwtPlot::xBottom)->title();
    title.setFont(QFont("Arial", 7, -1, false));
    m_plot->axisWidget(QwtPlot::xBottom)->setTitle(title);


    // legend
    QwtLegend *legend = new QwtLegend;
    legend->setFrameStyle(QFrame::NoFrame);
    m_plot->insertLegend(legend, QwtPlot::BottomLegend);

    // update timer
    m_timer.connect(&m_timer, SIGNAL(timeout()), m_plot, SLOT(replot()));
    m_timer.setSingleShot(false);
    m_timer.start(100);

    //zoomer
    QwtPlotZoomer* zoomer = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yRight, m_plot->canvas());
    zoomer->setMousePattern(QwtEventPattern::MouseSelect1, Qt::RightButton);
    zoomer->setMousePattern(QwtEventPattern::MouseSelect2, Qt::MidButton);

    //magnifier
    QwtPlotMagnifier* magnifier = new QwtPlotMagnifier(m_plot->canvas());
    magnifier->setMouseButton(Qt::NoButton);
    magnifier->setAxisEnabled(QwtPlot::yLeft, false);

    // panner
    QwtPlotPanner * panner = new QwtPlotPanner(m_plot->canvas());
    panner->setAxisEnabled(QwtPlot::yLeft, false);
    panner->setMouseButton(Qt::LeftButton);

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
    m_tools.push_back(split);
}

void GraphWidget::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> >stream){
    addStream(stream);
}


std::string GraphWidget::getName() const{
    std::stringstream title;
    title << implode(", ", m_seriesNames);
    return title.str();
}

