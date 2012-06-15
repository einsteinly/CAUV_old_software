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

#ifndef SHAPEDCLOCK_H
#define SHAPEDCLOCK_H

#include <QWidget>

class ShapedClock : public QWidget
{
    Q_OBJECT

public:
    ShapedClock(QWidget *parent = 0);
    QSize sizeHint() const;

public Q_SLOTS:
    void createSubmenu(QPoint p);

protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void focusOutEvent(QFocusEvent *event);

private:
    QPoint dragPosition;
};

#endif
