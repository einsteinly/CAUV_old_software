/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_BOOLEAN_DELEGATE_H__
#define __CAUV_GUI_BOOLEAN_DELEGATE_H__

#include <QtGui>

#include <delegates/delegate.h>

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
