/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_NUMERIC_DELEGATE_H__
#define __CAUV_NUMERIC_DELEGATE_H__

#include <QtGui>

#include <delegates/delegate.h>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class NumericDelegate : public AbstractNodeDelegate {
    Q_OBJECT
public:
    NumericDelegate(bool showTitle = true, QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    void setEditorData(QWidget *, const QModelIndex &) const;

    virtual QSize sizeHint(const QStyleOptionViewItem &option,
                           const QModelIndex &index) const;

    virtual QRect controlRect(const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

protected Q_SLOTS:
    void commit();

protected:
    bool m_titles;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_NUMERIC_DELEGATE_H__
