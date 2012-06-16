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

#ifndef __LIQUID_MAGMA_RADIAL_MENU_H__
#define __LIQUID_MAGMA_RADIAL_MENU_H__

#include <QAbstractItemView>
#include <QMap>

namespace liquid {
namespace magma {

class RadialSegment;
struct RadialSegmentStyle;
struct RadialMenuStyle;

class RadialMenu : public QAbstractItemView
{
    Q_OBJECT

public:
    RadialMenu(RadialMenuStyle const& style, QWidget *parent=0);

    virtual QRect visualRect ( const QModelIndex & index ) const;
    virtual void scrollTo(const QModelIndex&, QAbstractItemView::ScrollHint = EnsureVisible);
    virtual QModelIndex indexAt(const QPoint&) const;
    virtual QModelIndex moveCursor(QAbstractItemView::CursorAction, Qt::KeyboardModifiers);
    virtual int horizontalOffset() const;
    virtual int verticalOffset() const;
    virtual bool isIndexHidden(const QModelIndex&) const;
    virtual void setSelection(const QRect&, QFlags<QItemSelectionModel::SelectionFlag>);
    virtual QRegion visualRegionForSelection(const QItemSelection&) const;
    virtual void	setModel ( QAbstractItemModel * model );

public Q_SLOTS:
    void recomputeMask();
    void fitToContents();

protected:
    void resizeEvent(QResizeEvent *event);
    void focusOutEvent(QFocusEvent *event);
    void recomputeGeometry();
    int depthOfIndex(const QModelIndex& index);

    RadialMenuStyle const& m_style;
    QMap<QModelIndex, RadialSegment*> m_segmentMap;
};

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_RADIAL_MENU_H__
