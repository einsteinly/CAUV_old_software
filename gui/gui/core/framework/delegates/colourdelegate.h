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

#ifndef __CAUV_GUI_COLOUR_DELEGATE_H__
#define __CAUV_GUI_COLOUR_DELEGATE_H__

#include <QtGui>

#include <common/msg_classes/colour.h>

#include <framework/delegates/delegate.h>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class ColourDialog :  public QColorDialog {
    Q_OBJECT

    Q_PROPERTY(Colour colour READ colour WRITE setColour USER true)

public:
    ColourDialog(QWidget * parent = 0) : QColorDialog(parent){
    }

    ColourType::e type;

public Q_SLOTS:
    void setColour(Colour c){
        type = c.type;
        setCurrentColor(QColor::fromRgbF(c.r(), c.g(), c.b(), c.a()));
    }

    Colour colour() const {
        QColor c = currentColor();
        Colour colour = Colour::fromARGB(c.alphaF(),
                                         c.redF(),
                                         c.greenF(),
                                         c.blueF());
        colour.type = type;
        return colour;
    }
};

class ColourDelegate : public AbstractNodeDelegate {
    Q_OBJECT
public:
    ColourDelegate(QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    void setEditorData(QWidget *editor,
                       const QModelIndex &index) const;

    virtual QSize sizeHint(const QStyleOptionViewItem &option,
                           const QModelIndex &index) const;

    virtual QWidget * createEditor(QWidget *parent,
                                   const QStyleOptionViewItem &option,
                                   const QModelIndex &index) const;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_GUI_COLOUR_DELEGATE_H__
