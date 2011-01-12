#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QMdiArea>
#include <qwt_plot.h>
#include <qwt_plot_grid.h>
#include <qwt_series_data.h>

#include "../cauvinterfaceelement.h"
#include "../datastreamdragging.h"

namespace cauv {


    class DataStreamSeriesDataBase{};

    /**
      * A DataStreamSeriesData is used to bridge the interface between qwt and the cauv data stream classes
      * it uses a DataStreamRecorder to store samples
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


    /**
      * GraphArea - accepts data stream drops, adding them to its mdi area as it gets them
      *
      * @author Andy Pritchard
      */
    class GraphArea : public QMdiArea, public DataStreamDropListener, public CauvInterfaceElement {
        Q_OBJECT

    public:
        GraphArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual void initialise();
        void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
        void dropEvent(QDropEvent * event);
        void dragEnterEvent(QDragEnterEvent * event);
        void addGraph(cauv::GraphWidget * graph);
    };   
}

#endif // GRAPHWIDGET_H
