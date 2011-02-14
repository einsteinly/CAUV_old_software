#ifndef DATASTREAMRECORDER_H
#define DATASTREAMRECORDER_H

#include <QWidget>

#include <common/data_stream_tools.h>

class QSpinBox;

namespace Ui {
    class DataStreamRecorder;
}

namespace cauv {

    class DataStreamRecorderController : public QObject {
        Q_OBJECT

    public:
        DataStreamRecorderController(QSpinBox * samples);

    public Q_SLOTS:
        virtual void setNumSamples(int samples);
    };


    template <class T>
            class DataStreamRecorderView : public QWidget {
            public:
        DataStreamRecorderView(DataStreamRecorder<T> * recorder, QWidget *parent = 0);
        ~DataStreamRecorderView();

            private:
        Ui::DataStreamRecorder *ui;
    };

} // cauv

#endif // DATASTREAMRECORDER_H
