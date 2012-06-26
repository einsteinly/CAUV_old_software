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

QSize LabelPath::sizeHint() const{
    return QSize();
}

void LabelPath::paintEvent(QPaintEvent *){
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.fillRect(rect(), Qt::red);

    //painter.setPen(QPen(Qt::blue,5));

    painter.drawPath(m_path);

    int drawWidth = width() / 100;

    QFont font = painter.font();
    QPen pen = painter.pen();
    font.setPixelSize(drawWidth*2);
    painter.setFont(font);

    qreal totalWidth = fontMetrics().width(text());

    for ( int i = 0; i < text().size(); i++ ) {

        QString s = text();
        s.truncate(i);
        qreal partialWidth = fontMetrics().width(s);
        qreal percent = partialWidth / totalWidth;

        info() << percent;

        QPointF point = m_path.pointAtPercent(percent);
        qreal angle = m_path.angleAtPercent(percent);   // Clockwise is negative

        //qDebug() << "point" << point;
        //qDebug() << "angle" << angle;

        painter.save();
        // Move the virtual origin to the point on the curve
        painter.translate(point);
        // Rotate to match the angle of the curve
        // Clockwise is positive so we negate the angle from above
        painter.rotate(-angle);
        // Draw a line width above the origin to move the text above the line
        // and let Qt do the transformations
        painter.drawText(QPoint(0, -pen.width()),QString(text()[i]));
        painter.restore();
    }
}

void LabelPath::setPath(QPainterPath path){
    m_path = path;
}

QPainterPath LabelPath::getPath() const{
    return m_path;
}
