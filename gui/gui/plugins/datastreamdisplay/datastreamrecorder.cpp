#include "datastreamrecorder.h"

#include <QSpinBox>

using namespace cauv;

DataStreamRecorderView::DataStreamRecorderView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataStreamRecorder)
{
    ui->setupUi(this);
    ui->samples->connect(ui->samples, SIGNAL(valueChanged(int)), this, SLOT(setNumSamples(int)));
    ui->frequency->connect(ui->frequency, SIGNAL(valueChanged(int)), this, SLOT(setSampleFrequency(int)));
}

void DataStreamRecorderView::setNumSamples(int samples){
    foreach(boost::function<void(int)> update, m_sampleUpdateFunctions){
        update(samples);
    }
}

void DataStreamRecorderView::setSampleFrequency(int frequency){
    foreach(boost::function<void(int)> update, m_frequencyUpdateFunctions){
        update(frequency);
    }
}


DataStreamRecorderView::~DataStreamRecorderView()
{
    delete ui;
}
