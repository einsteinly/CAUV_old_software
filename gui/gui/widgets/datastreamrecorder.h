#ifndef DATASTREAMRECORDER_H
#define DATASTREAMRECORDER_H

#include <QWidget>

#include <common/data_stream_tools.h>

class QSpinBox;

namespace Ui {
    class DataStreamRecorder;
}

namespace cauv {

    class DataStreamRecorderView : public QWidget {
        Q_OBJECT

    public:
        DataStreamRecorderView(QWidget *parent = 0);
        ~DataStreamRecorderView();

        template<class T>
        void addRecorder(DataStreamRecorder<T> * recorder){
            m_updateFunctions.push_back(boost::bind(&DataStreamRecorder<T>::setNumSamples, recorder, _1));
        };

    protected:
        std::vector<boost::function<void(int)> > m_updateFunctions;

    protected Q_SLOTS:
        void setNumSamples(int samples);

    private:
        Ui::DataStreamRecorder *ui;
    };

} // cauv

#endif // DATASTREAMRECORDER_H
