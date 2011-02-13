#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QMdiArea>
#include <qwt_plot.h>
#include <qwt_plot_grid.h>
#include <qwt_series_data.h>

#include <common/data_stream.h>
#include <common/data_stream_tools.h>

#include "cauvinterfaceelement.h"
#include "datastreamdragging.h"

namespace cauv {


    class DataStreamSeriesDataBase{};

    /**
      * A DataStreamSeriesData is used to bridge the interface between qwt and the cauv data stream classes
      * it uses a DataStreamRecorder to store samples
      *
      * TODO: implement stubs
      *
      * @author Andy Pritchard
      */
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


    /**
      * GraphWidget
      *
      * @author Andy Pritchard
      */
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

        virtual ~GraphWidget(){
            std::cout << "destroyed GraphWidget" << std::endl;
        }

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
        std::set<std::string> seriesNames;
        std::set<boost::shared_ptr<DataStreamSeriesDataBase> > series;
        boost::shared_ptr<QwtPlotGrid> m_grid;
    }; 
}

#endif // GRAPHWIDGET_H
