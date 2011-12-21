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

namespace cauv {
    namespace gui {




        class Sampler : public QTimer {
            Q_OBJECT

        public:
            Sampler(){
                setSingleShot(false);
                connect(this, SIGNAL(timeout()), this, SLOT(sample()));
                start();
            }

        protected Q_SLOTS:
            virtual void sample() = 0;
        };


        template<class TTar, class TVal>
        class SamplingQueue : public Sampler, public QQueue<TVal>{

        public:
            SamplingQueue(TTar * target, int maxLength = 120, int sampleTime = 500) :
                m_maxLength(maxLength), m_sampleTime(sampleTime), m_target(target){
                setInterval(sampleTime);

                // fill queue
                for(int i = 0; i < maxLength; i++){
                    sample();
                }
            }

            virtual void sample(){
                QQueue<TVal>::enqueue(m_target->value());
                while (QQueue<TVal>::size() > m_maxLength) QQueue<TVal>::dequeue();
            }

        protected:
            int m_maxLength;
            int m_sampleTime;
            TTar * m_target;
        };



        class GraphingSpinBox : public QSpinBox {
            Q_OBJECT

        public:

            Q_PROPERTY(int value READ value WRITE setValue USER true)

            GraphingSpinBox(QWidget * parent = 0);

            QList<int> values() const;

            void paintEvent(QPaintEvent *);

        protected:
            SamplingQueue<GraphingSpinBox, int> m_samples;
        };



        class GraphingDoubleSpinBox : public QDoubleSpinBox {
            Q_OBJECT

        public:
            Q_PROPERTY(double value READ value WRITE setValue USER true)

            GraphingDoubleSpinBox(QWidget * parent = 0);

            QList<double> values() const;

            void paintEvent(QPaintEvent *);

        protected:
            SamplingQueue<GraphingDoubleSpinBox, double> m_samples;
        };

    } // namespace gui
} // namespace cauv

#endif // __CAUV_GRAPHBAR_H
