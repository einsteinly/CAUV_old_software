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

#include "datastreamrecorder.h"

#include <QSpinBox>
#include <utility/foreach.h>

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
