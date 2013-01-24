/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_COLOUR_DELEGATE_H__
#define __CAUV_GUI_COLOUR_DELEGATE_H__

#include <QtGui>

#include <common/msg_classes/colour.h>

#include <delegates/delegate.h>

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
