#include "datastreamrecorder.h"

#include <QSpinBox>

using namespace cauv;

DataStreamRecorderView::DataStreamRecorderView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataStreamRecorder)
{
    ui->setupUi(this);
    ui->samples->connect(ui->samples, SIGNAL(valueChanged(int)), this, SLOT(setNumSamples(int)));
}

void DataStreamRecorderView::setNumSamples(int samples){
    foreach(boost::function<void(int)> update, m_updateFunctions){
        update(samples);
    }
}


DataStreamRecorderView::~DataStreamRecorderView()
{
    delete ui;
}
