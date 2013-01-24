/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
