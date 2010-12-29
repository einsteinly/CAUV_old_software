#include "graphwidget.h"

#include <common/cauv_utils.h>
#include <common/data_stream_tools.h>

#include <boost/bind.hpp>

#include <QMdiSubWindow>
#include <QPen>
#include <QRectF>
#include <qwt_plot.h>
#include <qwt_legend.h>
#include <qwt_plot_grid.h>
#include <qwt_series_data.h>

using namespace cauv;

class DataStreamSeriesDataBase{};

template<class T>
class DataStreamSeriesData : public DataStreamSeriesDataBase, public QwtSeriesData<T>, public DataStreamRecorder<T> {

public:

    DataStreamSeriesData<T>(boost::shared_ptr<DataStream<T> >stream, unsigned int maximum) :
            DataStreamRecorder<T>(stream, maximum)
    {
    }

    virtual size_t size () const {
        return 1000; //m_history.size();
    }

    virtual T sample (size_t i) const {
        return sin(i/1000);//m_history[i];
    }

    virtual QRectF boundingRect () const {
        return QRectF(0, sample(0), size(), sample(size()));
    }
};


class GraphWidget : public QwtPlot, public DataStreamDropListener {

public:
    template<class T>
    explicit GraphWidget(boost::shared_ptr<DataStream<T> > stream) :
            m_grid(boost::make_shared<QwtPlotGrid>())
    {
        onStreamDropped(stream);
        this->setAcceptDrops(true);
        setupPlot();
    }

    void setupPlot() {
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

    void dropEvent(QDropEvent * event){
        DataStreamDropListener::dropEvent(event);
    }

    void dragEnterEvent(QDragEnterEvent * event){
        DataStreamDropListener::dragEnterEvent(event);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> >stream){
        // TODO: implement a data stream splitter for this
    }

    void onStreamDropped(boost::shared_ptr<DataStream<int> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<int8_t> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<float> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> >stream){
        boost::shared_ptr<DataStreamSplitter<floatYPR> > split = boost::make_shared<DataStreamSplitter<floatYPR> >(stream);
        addStream(split->yaw);
        addStream(split->pitch);
        addStream(split->roll);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> >stream){
        addStream(stream);
    }

    template<class T>
    void addStream(boost::shared_ptr<DataStream<T> > stream){
        // see if this series has already been registered
        if(seriesNames.end() == seriesNames.find(stream->getName())) {
            seriesNames.insert(stream->getName());
            series.insert(boost::make_shared<DataStreamSeriesData<T> >(stream, 1000));
            this->setTitle(QString::fromStdString(getName()));
        }
    }

    virtual std::string getName(){
        std::stringstream title;
        title << "[" << implode(", ", seriesNames) << "]";
        return title.str();
    }

protected:
    std::set<std::string> seriesNames;
    std::set<boost::shared_ptr<DataStreamSeriesDataBase> > series;
    boost::shared_ptr<QwtPlotGrid> m_grid;
};


GraphArea::GraphArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QMdiArea(parent),
        CauvInterfaceElement(name, auv, node) {
    this->setAcceptDrops(true);
}

void GraphArea::initialise(){
    m_actions->registerCentralView(this, name());
}

void GraphArea::dropEvent(QDropEvent * event){
    DataStreamDropListener::dropEvent(event);
}

void GraphArea::dragEnterEvent(QDragEnterEvent * event){
    DataStreamDropListener::dragEnterEvent(event);
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<int> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<float> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}

void GraphArea::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream){
    addSubWindow(new GraphWidget(stream))->show();
}
