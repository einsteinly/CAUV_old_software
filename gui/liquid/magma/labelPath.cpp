/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */



#include "labelPath.h"

#include <QtGui>

#include <debug/cauv_debug.h>

using namespace liquid;
using namespace liquid::magma;


LabelPath::LabelPath(QWidget *parent,
                     Qt::WindowFlags f):
    QLabel(parent, f){

}

LabelPath::LabelPath(const QString& text,
                     QWidget * parent,
                     Qt::WindowFlags f):
    QLabel(text, parent, f){

}

void LabelPath::paintEvent(QPaintEvent *){
    QPainter painter(this);
    painter.save();

    painter.setRenderHint(QPainter::Antialiasing);

    if(MAGMA_DEBUG_LAYOUT)
        painter.fillRect(rect(), QColor(255,0,0,125));

    float rotation = (int)m_rotation % 360;

    rotation = (rotation > 270) ? rotation - 180 : rotation;
    rotation = (rotation <= 90) ? rotation : rotation + 180;

    painter.translate(width()/2, height()/2);
    painter.rotate(-rotation);

    qreal scale = (qreal)fontMetrics().width(text())/(qreal)getTextWidth();
    painter.scale(1.0/scale, 1.0/scale);

    float textHeight = fontMetrics().height() * scale;
    float textWidth = getTextWidth() * scale;
    QRect textRect(-textWidth/2, -textHeight/2, textWidth, textHeight);

    if(MAGMA_DEBUG_LAYOUT)
        painter.fillRect(textRect, QColor(0,0,255,125));

    painter.drawText(textRect, text(), QTextOption(Qt::AlignCenter));

    painter.restore();
}

void LabelPath::setRotation(float rot){
    m_rotation = rot;
    resize(sizeHint());
}

float LabelPath::getRotation() const{
    return m_rotation;
}

void LabelPath::setTextWidth(float width){
    m_textWidth = width;
    resize(sizeHint());
}

float LabelPath::getTextWidth() const{
    return m_textWidth;
}

QSize LabelPath::sizeHint() const {
    QTransform transform;
    // rotate around centre
    transform.translate(getTextWidth()/2, fontMetrics().height()/2);
    transform = transform.rotate(m_rotation);
    QRect rotatedRect = transform.mapRect(
                QRect(0,0,getTextWidth(),fontMetrics().height()));
    return rotatedRect.size();
}
