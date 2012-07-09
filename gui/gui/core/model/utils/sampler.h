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

#ifndef __CAUV_SAMPLEQUEUE_H_
#define __CAUV_SAMPLEQUEUE_H_

#include <QQueue>
#include <QTimer>

#include <boost/function.hpp>

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


    template<class T>
    class SampleQueue : public Sampler, public QQueue<T>{

    public:
        SampleQueue(boost::function<T()> getSample, int maxLength = 120, int sampleTime = 500) :
            m_maxLength(maxLength), m_sampleTime(sampleTime), m_getSample(getSample){
            setInterval(sampleTime);

            // fill queue
            for(int i = 0; i < maxLength; i++){
                sample();
            }
        }

        virtual void sample(){
            enqueue(m_getSample());
            while (QQueue<T>::size() > m_maxLength) QQueue<T>::dequeue();
        }

        const QList<T>& samples() {
            return *(this);
        }

    protected:
        int m_maxLength;
        int m_sampleTime;
        boost::function <T()> m_getSample;
    };


    } // namesapce gui
} // namespace cauv

#endif // __CAUV_SAMPLEQUEUE_H_
