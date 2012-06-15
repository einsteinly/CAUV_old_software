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

#include <QWidget>

namespace liquid {
namespace magma {

class RadialMenu : public QWidget
{
    Q_OBJECT

public:
    RadialMenu(QWidget *parent = 0);
    QSize sizeHint() const;

public Q_SLOTS:
    void createSubmenu(QPoint p);

protected:
    void resizeEvent(QResizeEvent *event);
    void focusOutEvent(QFocusEvent *event);
    void recomputeMask();

private:
    QPoint dragPosition;
};

} // namespace magma
} // namespace liquid

#endif // namespace __LIQUID_MAGMA_RADIAL_MENU_H__
