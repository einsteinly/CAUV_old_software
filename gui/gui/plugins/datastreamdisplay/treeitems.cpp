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

#include "treeitems.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cauv;

DataStreamTreeItemBase::DataStreamTreeItemBase(boost::shared_ptr<DataStreamBase> stream, QTreeWidgetItem * parent):
        QTreeWidgetItem(parent), m_stream(stream){
    if(stream->isMutable()) {
        setTextColor(1, QColor::fromRgb(52, 138, 52));
    }

    qRegisterMetaType<Image>("Image");

    connect(this, SIGNAL(iconUpdated(int, Image)), this, SLOT(updateIcon(int, Image)));
    connect(this, SIGNAL(valueUpdated(QString)), this, SLOT(updateValue(QString)));
}

void DataStreamTreeItemBase::updateIcon(int cell, QImage &image){
    this->setIcon(cell, QIcon(QPixmap::fromImage(image)));
}

void DataStreamTreeItemBase::updateIcon(int cell, const Image &image){
    try {
        cv::Mat mat_rgb;
        cv::cvtColor(image.mat(), mat_rgb, CV_BGR2RGB);

        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                               mat_rgb.rows, QImage::Format_RGB888);

        this->updateIcon(cell, qImage);

    } catch (...){
        error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__;
    }
}

void DataStreamTreeItemBase::updateValue(const QString value) {
    // no update if editing
    if(!(this->flags() & Qt::ItemIsEditable))
        this->setText(1, value);
}

boost::shared_ptr<DataStreamBase> DataStreamTreeItemBase::getDataStreamBase(){
    return m_stream;
}




template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
}

template<> void DataStreamTreeItem<Image>::onChange(const Image value){
    // Add a fancy icon in the data stream list with the current view from the camera.
    Q_EMIT this->iconUpdated(1, value);
}

template<> void DataStreamTreeItem<floatYPR>::onChange(const floatYPR value){
    std::stringstream stream;
    stream.flags(std::stringstream::fixed);
    stream.precision(2);
    stream << "[ y = " << value.yaw << ", p = "<< value.pitch << ", r = "<< value.roll << " ]";
    Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
}

template<> void DataStreamTreeItem<float>::onChange(const float value){
    std::stringstream stream;
    stream.flags(std::stringstream::fixed);
    stream.precision(2);
    stream << value;
    Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
}

template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant &value){
    return boost::lexical_cast<int>(value.toString().toStdString());
}

template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& ){
    // TODO: implement, should recognise something like (1.0, 2.0, 3.0)
    return floatYPR();
}

template<> MotorDemand DataStreamTreeItem<MotorDemand>::qVariantToValue(QVariant& ){
    // this shouldn't ever be used
    throw new boost::bad_lexical_cast;
}

template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& ){
    // this shouldn't ever be used
    throw new boost::bad_lexical_cast;
}

