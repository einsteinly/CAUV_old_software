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

#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QPointF>
#include <QTimer>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <utility/data_stream.h>
#include <utility/data_stream_tools.h>

#include <gui/core/datastreamdragging.h>

#include <qwt_series_data.h>
#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>

#include "datastreamrecorder.h"
#include "datastreamdisplay/ui_graphs.h"


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
             // times 2 as we plot 2 points for each sample
            // this creates the step effect
            return (this->m_history.size())*2;
        }

        float toTime(boost::posix_time::ptime epoch, boost::posix_time::ptime time) const{
            boost::posix_time::time_duration delta = epoch - time;
            return ((float)delta.ticks())/(float)delta.ticks_per_second();
        }

        QPointF sample (size_t i) const {

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
            return QPointF(-seconds, this->m_history[sample]);
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
        GraphWidget(boost::shared_ptr<DataStream<T> > stream):
                m_plot(new QwtPlot()), ui(new Ui::GraphWidget()), m_recorderView(new DataStreamRecorderView())
        {
            ui->setupUi(this);
            ui->optionsWidget->hide();
            onStreamDropped(stream);
            this->setAcceptDrops(true);
            setupPlot();
            ui->options->addWidget(m_recorderView);
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
        std::vector<boost::shared_ptr<DataStreamTool> > m_tools;
    }; 

} // namespace cauv

#endif // GRAPHWIDGET_H
