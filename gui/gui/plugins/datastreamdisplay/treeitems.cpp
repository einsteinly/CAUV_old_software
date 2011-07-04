#include "treeitems.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cauv;
using namespace cauv::gui;

NodeTreeItemBase::NodeTreeItemBase(boost::shared_ptr<NodeBase> node, QTreeWidgetItem * parent):
        QTreeWidgetItem(parent), m_node(node){
    this->setText(0, QString::fromStdString(node->nodeName(false)));
    if(node->isMutable()) {
        setTextColor(1, QColor::fromRgb(52, 138, 52));
    }
}

void NodeTreeItemBase::updateIcon(int cell, QImage &image){
    this->setIcon(cell, QIcon(QPixmap::fromImage(image)));
}

void NodeTreeItemBase::updateIcon(int cell, const Image &image){
    try {
        cv::Mat mat_rgb;
        cv::cvtColor(image.cvMat(), mat_rgb, CV_BGR2RGB);

        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                               mat_rgb.rows, QImage::Format_RGB888);

        this->updateIcon(cell, qImage);

    } catch (...){
        error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__;
    }
}

void NodeTreeItemBase::updateValue(const QString value) {
    // no update if editing
    if(!(this->flags() & Qt::ItemIsEditable))
        this->setText(1, value);
}

boost::shared_ptr<NodeBase> NodeTreeItemBase::getNode(){
    return m_node;
}




/*
template<> void NodeTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
}

template<> void DataStreamTreeItem<Image>::onChange(const Image value){
    // Add a fancy icon in the data stream list with the current view from the camera.
    Q_EMIT this->iconUpdated(1, value);
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

*/
