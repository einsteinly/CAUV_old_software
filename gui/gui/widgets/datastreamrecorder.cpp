#include "datastreamrecorder.h"
#include "ui_datastreamrecorder.h"

#include <QSpinBox>

using namespace cauv;



DataStreamRecorderController::DataStreamRecorderController(QSpinBox * samples){
    samples->connect(samples, SIGNAL(valueChanged(int)), this, SLOT(setNumSamples(int)));
}

void DataStreamRecorderController::setNumSamples(int samples){

}


template<class T>
DataStreamRecorderView<T>::DataStreamRecorderView(DataStreamRecorder<T> * recorder, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataStreamRecorder)
{
    ui->setupUi(this);
}


template<class T>
DataStreamRecorderView<T>::~DataStreamRecorderView()
{
    delete ui;
}
