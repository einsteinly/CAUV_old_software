#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QPointF>
#include <QTimer>
#include <QDropEvent>
#include <QDragEnterEvent>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "../model/variants.h"
#include "../datastreamdragging.h"
#include "../utils/noderecorder.h"

#include <qwt_series_data.h>
#include <qwt_plot.h>

#include "ui_graphs.h"


namespace Ui {
    class GraphWidget;
}

namespace cauv {
    namespace gui {

        class NumericNode;

        class DataStreamSeriesData : public QObject, public QwtSeriesData<QPointF>, public DataRecorder<numeric_variant_t> {
        Q_OBJECT

        public:
            DataStreamSeriesData(boost::shared_ptr<NumericNode> node, unsigned int maximum);
            size_t size () const;
            float toTime(boost::posix_time::ptime epoch, boost::posix_time::ptime time) const;
            QPointF sample (size_t i) const;
            QRectF boundingRect () const;

        public Q_SLOTS:
            void change(numeric_variant_t value);

        protected:
            numeric_variant_t m_max;
            numeric_variant_t m_min;
        };


        class GraphWidget : public QWidget, public NodeDropListener {
        public:

            static const QColor colours[];

            GraphWidget();
            GraphWidget(boost::shared_ptr<NumericNode> node);
            ~GraphWidget();

            QSize sizeHint() const;
            void setupPlot();
            void dropEvent(QDropEvent * event);
            void dragEnterEvent(QDragEnterEvent * event);

            void onNodeDropped(boost::shared_ptr<NumericNode>);

            void addNode(boost::shared_ptr<NumericNode > node);
            virtual std::string getName() const;

        protected:
            typedef std::map<std::string, boost::shared_ptr<NumericNode> > series_map_t;
            series_map_t m_nodes;
            QTimer m_timer;
            QwtPlot * m_plot;
            Ui::GraphWidget * ui;
        };
    } // namespace gui
} // namespace cauv

#endif // GRAPHWIDGET_H
