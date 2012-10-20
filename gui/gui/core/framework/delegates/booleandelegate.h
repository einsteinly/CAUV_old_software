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

#ifndef __CAUV_GUI_BOOLEAN_DELEGATE_H__
#define __CAUV_GUI_BOOLEAN_DELEGATE_H__

#include <QtGui>

#include <framework/delegates/delegate.h>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {


class BooleanDelegate : public AbstractNodeDelegate {
    Q_OBJECT
public:
    BooleanDelegate(QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    QWidget * createEditor(QWidget *parent,
                           const QStyleOptionViewItem &option,
                           const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;

    QRect controlRect(const QStyleOptionViewItem &option,
                      const QModelIndex &index) const;

protected Q_SLOTS:
    void commit();
};

} // namespace gui
} // namespace cauv


#endif // __CAUV_GUI_BOOLEAN_DELEGATE_H__
