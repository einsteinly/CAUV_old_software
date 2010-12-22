#include "graphwidget.h"
#include "ui_graphwidget.h"

#include <boost/bind.hpp>
#include <QMdiSubWindow>
#include <qwt_plot.h>



class GraphWidget : public QwtPlot, public DataStreamDropListener {

public:
    GraphWidget() {
        this->setTitle("Blank Graph");
        this->setAcceptDrops(true);
    }

    void dropEvent(QDropEvent * event){
        DataStreamDropListener::dropEvent(event);
    }

    void dragEnterEvent(QDragEnterEvent * event){
        DataStreamDropListener::dragEnterEvent(event);
    }

    template<class T>
    explicit GraphWidget(boost::shared_ptr<DataStream<T> > stream) {
        addStream<T>(stream);
        this->setAcceptDrops(true);
    }

    /*void onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<int> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<int8_t> >stream){
        addStream(stream);
    }

    void onStreamDropped(boost::shared_ptr<DataStream<float> >stream){
        addStream(stream);
    }*/

    template<class T>
    void onStreamDropped(boost::shared_ptr<DataStream<T> >stream){
        addStream(stream);
    }

    template<class T>
    void addStream(boost::shared_ptr<DataStream<T> > stream){
        stream->onUpdate.connect(boost::bind( static_cast<void (GraphWidget::*)(std::string, T)>(&GraphWidget::add), this, stream->getName(), _1));
        if(series.end() == series.find(stream->getName()))
            series.insert(stream->getName());

        this->setTitle(QString::fromStdString(getName()));
    }

    virtual std::string getName(){
        std::stringstream title;
        title << "[";
        std::set<std::string>::iterator i;
        for (i=series.begin(); i!=series.end(); i++){
            title << *i << ", ";
        }
        title << "]";

        return title.str();
    }

    virtual void add(std::string series, int8_t data){

    }

    virtual void add(std::string series, int data){

    }

    virtual void add(std::string series, float data){

    }

    virtual void add(std::string series, autopilot_params_t data){

    }

protected:
    std::set<std::string> series;

};


GraphArea::GraphArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent) :
        QMdiArea(parent),
        CauvInterfaceElement(name, auv) {
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
