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

#ifndef __LIQUID_MAGMA_RADIAL_SEGMENT_H__
#define __LIQUID_MAGMA_RADIAL_SEGMENT_H__

#include <QtGui>

namespace liquid {
namespace magma {

struct RadialSegmentStyle;
class RadialMenuItem;

class RadialSegment : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(float radius READ getRadius WRITE setRadius)
    Q_PROPERTY(float rotation READ getRotation WRITE setRotation)
    Q_PROPERTY(float angle READ getAngle WRITE setAngle)

public:
    RadialSegment(RadialSegmentStyle const& style,
                  bool isRoot = false,
                  QWidget *parent = 0
            );
    QSize sizeHint() const;

    QRegion recomputeMask();

    void setRadius(float radius);
    float getRadius() const;

    void setRotation(float rotation);
    float getRotation() const;

    void setAngle(float angle);
    float getAngle() const;

    void addItem(RadialMenuItem * );
    void removeItem(RadialMenuItem * );
    void relayoutItems();

    const RadialMenuItem* itemAt(QPoint const& p) const;

    void animateIn();

protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

    RadialSegmentStyle const& m_style;
    float m_radius;
    float m_rotation;
    float m_angle;

    bool m_animationsSetup;

    QParallelAnimationGroup m_startupAnimations;
    QParallelAnimationGroup m_runningAnimations;

    QList<RadialMenuItem * > m_items;

    bool m_isRoot;

protected Q_SLOTS:
    void itemHovered();
    void itemSelected();

Q_SIGNALS:
    void maskComputed(QRegion);
    void sizeChanged(QSize);
    void itemHovered(RadialMenuItem *);
    void itemSelected(RadialMenuItem *);

private:
    QPoint dragPosition;
};

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_RADIAL_SEGMENT_H__
