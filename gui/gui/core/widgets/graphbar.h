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

#ifndef __CAUV_GRAPHBAR_H
#define __CAUV_GRAPHBAR_H

#include <QSpinBox>

#include <QQueue>
#include <QTimer>

#include <debug/cauv_debug.h>

#include <gui/core/model/utils/sampler.h>

namespace cauv {
    namespace gui {

        class GraphingSpinBox : public QSpinBox {
            Q_OBJECT

        public:

            Q_PROPERTY(int value READ value WRITE setValue USER true)

            GraphingSpinBox(QWidget * parent = 0);

            void setSampler(boost::shared_ptr<SampleQueue<QVariant> > sampler);

            boost::shared_ptr<SampleQueue<QVariant> > sampler() const;

            void paintEvent(QPaintEvent *);

        protected:
            boost::shared_ptr<SampleQueue<QVariant> > m_sampler;
        };



        class GraphingDoubleSpinBox : public QDoubleSpinBox {
            Q_OBJECT

        public:
            Q_PROPERTY(double value READ value WRITE setValue USER true)

            GraphingDoubleSpinBox(QWidget * parent = 0);

            void setSampler(boost::shared_ptr<SampleQueue<QVariant> > sampler);

            boost::shared_ptr<SampleQueue<QVariant> > sampler() const;

            void paintEvent(QPaintEvent *);

        protected:
            boost::shared_ptr<SampleQueue<QVariant> > m_sampler;
        };

    } // namespace gui
} // namespace cauv

#endif // __CAUV_GRAPHBAR_H
