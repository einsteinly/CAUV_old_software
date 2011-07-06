#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QPointF>
#include <QTimer>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <gui/core/datastreamdragging.h>
#include <gui/core/model/noderecorder.h>
#include <gui/core/model/nodes.h>

#include <qwt_series_data.h>
#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>

#include "datastreamdisplay/ui_graphs.h"


namespace cauv {
    namespace gui {

        /**
      * A DataStreamSeriesData is used to bridge the interface between qwt and the cauv data stream classes
      * it uses a DataStreamRecorder to store samples
      **
      * @author Andy Pritchard
      */
        class DataStreamSeriesData : public QObject, public QwtSeriesData<QPointF>, public DataRecorder<numeric_variant_t> {
        Q_OBJECT

        public:

            DataStreamSeriesData(boost::shared_ptr<NumericNode> node, unsigned int maximum) :
                    DataRecorder<numeric_variant_t>(maximum), m_max(0), m_min(0) {
                node->connect(node.get(), SIGNAL(onUpdate(numeric_variant_t)), this, SLOT(change(numeric_variant_t)));
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
                return QPointF(-seconds, boost::apply_visitor(to_float(), this->m_history[sample]));
            }

            QRectF boundingRect () const {
                if(this->m_history.empty())
                    return QRectF(-60, 0, 60, 10);
                else {
                    // show the last 60 seconds;
                    return QRectF(-60, boost::apply_visitor(to_float(), m_min), 60,
                                       boost::apply_visitor(to_float(), m_max) -
                                       boost::apply_visitor(to_float(), m_min));
                }
            }

        public Q_SLOTS:

            void change(numeric_variant_t value){
                if(m_max < value)
                    m_max = value;
                if(value < m_min)
                    m_min = value;
                DataRecorder<numeric_variant_t>::update(value);
            }



        protected:
            numeric_variant_t m_max;
            numeric_variant_t m_min;
        };




        /**
      * GraphWidget
      *
      * @author Andy Pritchard
      */
        class GraphWidget : public QWidget, public NodeDropListener {
        public:

            static const QColor colours[];

            GraphWidget(boost::shared_ptr<NumericNode> node):
                    m_plot(new QwtPlot()), ui(new Ui::GraphWidget())
            {
                ui->setupUi(this);
                ui->optionsWidget->hide();
                onNodeDropped(node);
                this->setAcceptDrops(true);
                setupPlot();
            }


            ~GraphWidget();

            QSize sizeHint() const;
            void setupPlot();
            void dropEvent(QDropEvent * event);
            void dragEnterEvent(QDragEnterEvent * event);

            void onNodeDropped(boost::shared_ptr<NumericNode>);

            void addNode(boost::shared_ptr<NumericNode > node);
            virtual std::string getName() const;

        protected:
            std::set<std::string> m_seriesNames;
            QTimer m_timer;
            QwtPlot * m_plot;
            Ui::GraphWidget * ui;
        };
    } // namespace gui
} // namespace cauv

#endif // GRAPHWIDGET_H
