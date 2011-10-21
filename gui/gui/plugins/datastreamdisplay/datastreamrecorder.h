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

#ifndef DATASTREAMRECORDER_H
#define DATASTREAMRECORDER_H

#include <QWidget>

#include <common/data_stream_tools.h>

#include "datastreamdisplay/ui_datastreamrecorder.h"

class QSpinBox;

namespace cauv {

    class DataStreamRecorderView : public QWidget {
        Q_OBJECT

    public:
        DataStreamRecorderView(QWidget *parent = 0);
        ~DataStreamRecorderView();

        template<class T>
        void addRecorder(DataStreamRecorder<T> * recorder){
            m_sampleUpdateFunctions.push_back(boost::bind(&DataStreamRecorder<T>::setNumSamples, recorder, _1));
            m_frequencyUpdateFunctions.push_back(boost::bind(&DataStreamRecorder<T>::setSampleFrequency, recorder, _1));
        };

    protected:
        std::vector<boost::function<void(int)> > m_sampleUpdateFunctions;
        std::vector<boost::function<void(int)> > m_frequencyUpdateFunctions;

    protected Q_SLOTS:
        void setNumSamples(int samples);
        void setSampleFrequency(int samples);

    private:
        Ui::DataStreamRecorder *ui;
    };

} // cauv

#endif // DATASTREAMRECORDER_H
