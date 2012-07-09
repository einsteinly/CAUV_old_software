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

#ifndef __CAUV_QWTGRAPHWIDGET_H__
#define __CAUV_QWTGRAPHWIDGET_H__

#include <QPointF>
#include <QTimer>
#include <QDropEvent>
#include <QDragEnterEvent>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <gui/core/model/variants.h>
#include <gui/core/model/utils/datarecorder.h>
#include <gui/core/nodedragging.h>

#include <qwt_series_data.h>
#include <qwt_plot.h>


namespace Ui {
    class GraphWidget;
}

namespace cauv {
    namespace gui {
/*
        class NumericNodeBase;

        class DataStreamSeriesData : public QObject, public QwtSeriesData<QPointF> {//, public DataRecorder<float> {
        Q_OBJECT

        public:
            DataStreamSeriesData(boost::shared_ptr<NumericNodeBase> const& node, unsigned int maximum);
            size_t size () const;
            float toTime(boost::posix_time::ptime epoch, boost::posix_time::ptime time) const;
            QPointF sample (size_t i) const;
            QRectF boundingRect () const;

        public Q_SLOTS:
            void change(float value);

        protected:
            float m_max;
            float m_min;
        };


        class GraphWidget : public QWidget, public NodeDropListener {
        public:

            static const QColor colours[];

            GraphWidget();
            GraphWidget(boost::shared_ptr<NumericNodeBase> const&);
            //~GraphWidget();

            void setupPlot();
            bool accepts(boost::shared_ptr<Node> const&);

            void onNodeDropped(boost::shared_ptr<Node> const&);

            void addNode(boost::shared_ptr<NumericNodeBase >  const&);
            virtual std::string getName() const;

        protected:
            typedef std::map<std::string, boost::shared_ptr<NumericNodeBase> > series_map_t;
            series_map_t m_nodes;
            QTimer m_timer;
            QwtPlot * m_plot;
            //Ui::GraphWidget * ui;
        };*/
    } // namespace gui
} // namespace cauv

#endif // __CAUV_QWTGRAPHWIDGET_H__
