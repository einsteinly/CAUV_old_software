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

#ifndef __LIQUID_MAGMA_LABEL_PATH_H__
#define __LIQUID_MAGMA_LABEL_PATH_H__

#include <QtGui>

namespace liquid {
namespace magma {

class LabelPath : public QLabel
{
    Q_OBJECT
    //Q_PROPERTY(QPainterPath path READ getPath WRITE setPath)

public:
    LabelPath(QWidget * parent = 0,
              Qt::WindowFlags f = 0
            );

    LabelPath(const QString& text,
              QWidget * parent = 0,
              Qt::WindowFlags f = 0
            );
    QSize sizeHint() const;

    void setRotation(float rotation);
    float getRotation() const;

    void setTextWidth(float width);
    float getTextWidth() const;

protected:

    void paintEvent(QPaintEvent *);

    float m_rotation;
    float m_textWidth;
};

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_LABEL_PATH_H__
