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
