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

#ifndef __CAUV_NUMERIC_DELEGATE_H__
#define __CAUV_NUMERIC_DELEGATE_H__

#include <QtGui>

#include <gui/core/framework/delegates/delegate.h>

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
