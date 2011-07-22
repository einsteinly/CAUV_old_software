#include "graphs.h"
#include "datastreamdisplay/ui_graphs.h"

#include <common/cauv_utils.h>

#include <gui/core/model/nodes/numericnode.h>

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
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>

using namespace cauv;
using namespace cauv::gui;


const QColor GraphWidget::colours[] = {
    Qt::darkBlue, Qt::red, Qt::darkGreen,  Qt::yellow,
    Qt::darkCyan, Qt::darkGray, Qt::darkMagenta, Qt::darkRed, Qt::darkYellow,
    Qt::blue, Qt::cyan, Qt::gray, Qt::green, Qt::magenta,
};


DataStreamSeriesData::DataStreamSeriesData(boost::shared_ptr<NumericNode> node, unsigned int maximum) :
        DataRecorder<numeric_variant_t>(maximum), m_max(0), m_min(0) {
    node->connect(node.get(), SIGNAL(onUpdate(numeric_variant_t)), this, SLOT(change(numeric_variant_t)));
}

size_t DataStreamSeriesData::size () const {
    // times 2 as we plot 2 points for each sample
    // this creates the step effect
    return (this->m_history.size())*2;
}

float DataStreamSeriesData::toTime(boost::posix_time::ptime epoch, boost::posix_time::ptime time) const{
    boost::posix_time::time_duration delta = epoch - time;
    return ((float)delta.ticks())/(float)delta.ticks_per_second();
}

QPointF DataStreamSeriesData::sample (size_t i) const {

    // as each sample is represented by two points the actual sample we're plotting is
    // at half of the total plot size
    size_t sample = (i>>1);

    // for even samples we use the time at "sample" otherwise we use the
    // time of the next sample (to show it as a step change instead of
    // ramping to it)
    float seconds = toTime(boost::posix_time::microsec_clock::local_time(), this->m_timestamps[sample + (i&0x01)]);

    // the very last point should be pegged at zero seconds so that it's stretched
    if(i == (this->m_history.size()*2)-1){
        seconds = 0.0;
    }

    // times are shown as negative in seconds from the current time
    return QPointF(-seconds, boost::apply_visitor(cast_to<float>(), this->m_history[sample]));
}

QRectF DataStreamSeriesData::boundingRect () const {
    if(this->m_history.empty())
        return QRectF(-60, 0, 60, 10);
    else {
        // show the last 60 seconds;
        return QRectF(-60, boost::apply_visitor(cast_to<float>(), m_min), 60,
                      boost::apply_visitor(cast_to<float>(), m_max) -
                      boost::apply_visitor(cast_to<float>(), m_min));
    }
}

void DataStreamSeriesData::change(numeric_variant_t value){
    if(m_max < value)
        m_max = value;
    if(value < m_min)
        m_min = value;
    DataRecorder<numeric_variant_t>::update(value);
}




GraphWidget::GraphWidget():
        m_plot(new QwtPlot()), ui(new Ui::GraphWidget())
{
    ui->setupUi(this);
    ui->optionsWidget->hide();
    this->setAcceptDrops(true);
    setupPlot();
}

GraphWidget::GraphWidget(boost::shared_ptr<NumericNode> node):
        m_plot(new QwtPlot()), ui(new Ui::GraphWidget())
{
    ui->setupUi(this);
    ui->optionsWidget->hide();
    onNodeDropped(node);
    this->setAcceptDrops(true);
    setupPlot();
}

void GraphWidget::addNode(boost::shared_ptr<NumericNode> node){
    // see if this series has already been registered
    if(m_seriesNames.end() == m_seriesNames.find(node->nodeName())) {
        // series data
        DataStreamSeriesData * series = new DataStreamSeriesData(node, 1000);
        m_seriesNames.insert(node->nodeName());

        // series plotter
        std::stringstream str;
        str << node->nodeName();
        if(node->getUnits().length() > 0)
            str << " (" <<  node->getUnits() << ")";
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
    NodeDropListener::dropEvent(event);
}

void GraphWidget::dragEnterEvent(QDragEnterEvent * event){
    NodeDropListener::dragEnterEvent(event);
}

void GraphWidget::onNodeDropped(boost::shared_ptr<NumericNode> node){
    addNode(node);
}

std::string GraphWidget::getName() const{
    std::stringstream title;
    title << implode(", ", m_seriesNames);
    return title.str();
}

