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

#include <QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QModelIndex>

namespace liquid {
namespace magma {

struct RadialSegmentStyle;

class RadialSegment : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(float radius READ getRadius WRITE setRadius)
    Q_PROPERTY(float rotation READ getRotation WRITE setRotation)
    Q_PROPERTY(float angle READ getAngle WRITE setAngle)

public:
    RadialSegment(RadialSegmentStyle const& style, float radius = 50,
                  float roation = 0, float angle = 150, QWidget *parent = 0);
    QSize sizeHint() const;

    QRegion recomputeMask();

    void setRadius(float radius);
    float getRadius();

    void setRotation(float rotation);
    float getRotation();

    void setAngle(float angle);
    float getAngle();

    QModelIndex indexAt(QPoint const& p);

protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void showEvent(QShowEvent *);

    RadialSegmentStyle const& m_style;
    float m_radius;
    float m_rotation;
    float m_angle;

    QParallelAnimationGroup m_startupAnimations;
    QParallelAnimationGroup m_runningAnimations;

protected Q_SLOTS:
    void animateIn();

Q_SIGNALS:
    void maskComputed(QRegion);

private:
    QPoint dragPosition;
};

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_RADIAL_SEGMENT_H__
