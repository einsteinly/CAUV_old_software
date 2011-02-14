#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QPointF>
#include <QTimer>

#include "ui_graphs.h"
#include "datastreamdragging.h"
#include "widgets/datastreamrecorder.h"

#include <qwt_series_data.h>
#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>

#include <common/data_stream.h>
#include <common/data_stream_tools.h>

#include <boost/date_time/posix_time/posix_time.hpp>


namespace Ui {
    class GraphWidget;
}

namespace cauv {


    class DataStreamSeriesDataBase{};

    /**
      * A DataStreamSeriesData is used to bridge the interface between qwt and the cauv data stream classes
      * it uses a DataStreamRecorder to store samples
      **
      * @author Andy Pritchard
      */
    template<class T>
    class DataStreamSeriesData : public DataStreamSeriesDataBase, public QwtSeriesData<QPointF>, public DataStreamRecorder<T> {

    public:

        DataStreamSeriesData<T>(boost::shared_ptr<DataStream<T> >stream, unsigned int maximum) :
                DataStreamRecorder<T>(stream, maximum), m_max(0), m_min(0) {
        }

        void change(const T &data, const boost::posix_time::ptime timestamp){
            if(m_max < data)
                m_max = data;
            if(m_min > data)
                m_min = data;
            DataStreamRecorder<T>::change(data, timestamp);
        }

        size_t size () const {
            return this->m_history.size();
        }

        float toTime(boost::posix_time::ptime epoch, boost::posix_time::ptime time) const{
            boost::posix_time::time_duration delta = epoch - time;
            return ((float)delta.ticks())/(float)delta.ticks_per_second();
        }

        QPointF sample (size_t i) const {
            float seconds = toTime(boost::posix_time::microsec_clock::local_time(), this->m_timestamps[i]);

            if(i == this->m_history.size()-1){
                seconds = 0.0;
            }

            return QPointF(-seconds, this->m_history[i]);
        }

        QRectF boundingRect () const {
            if(this->m_history.empty())
                return QRectF(-60, 0, 60, 10);
            else {
                // show the last 60 seconds
                return QRectF(-60, (float)m_min, 60, (float)(m_max-m_min));
            }
        }

    protected:
        T m_max;
        T m_min;
    };




    /**
      * GraphWidget
      *
      * @author Andy Pritchard
      */
    class GraphWidget : public QWidget, public DataStreamDropListener {
    public:


        static const QColor colours[];


        template<class T>
        explicit GraphWidget(boost::shared_ptr<DataStream<T> > stream):
                m_plot(new QwtPlot()), ui(new Ui::GraphWidget()), m_recorderView(new DataStreamRecorderView())
        {
            ui->setupUi(this);
            onStreamDropped(stream);
            this->setAcceptDrops(true);
            setupPlot();
            ui->widgets->addWidget(m_recorderView);
        }

        ~GraphWidget();

        QSize sizeHint() const;
        void setupPlot();
        void dropEvent(QDropEvent * event);
        void dragEnterEvent(QDragEnterEvent * event);

        void onStreamDropped(boost::shared_ptr<DataStream<int> >stream);
        void onStreamDropped(boost::shared_ptr<DataStream<int8_t> >stream);
        void onStreamDropped(boost::shared_ptr<DataStream<float> >stream);
        void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> >stream);
        void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> >stream);

        template<class T> void addStream(boost::shared_ptr<DataStream<T> > stream);
        virtual std::string getName() const;

    protected:
        std::set<std::string> m_seriesNames;
        QTimer m_timer;
        QwtPlot * m_plot;
        Ui::GraphWidget * ui;
        DataStreamRecorderView * m_recorderView;
    }; 

} // namespace cauv

#endif // GRAPHWIDGET_H
