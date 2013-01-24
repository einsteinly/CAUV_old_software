/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_LABEL_H__
#define __LIQUID_LABEL_H__

#include <QLabel>
#include <QPalette>

namespace liquid{

// trivial subclass: force transparent background, no interaction
class LiquidLabel: public QLabel{
    public:
        LiquidLabel(QString text, QWidget* parent=0)
            : QLabel(text, parent){

            setTextInteractionFlags(Qt::NoTextInteraction);

            setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            
            QPalette transparent_bg = palette();
            for(int i=0; i < QPalette::NColorGroups; i++){
                 QColor color = transparent_bg.brush(QPalette::ColorGroup(i), QPalette::Window).color();
                 color.setAlpha(0);
                 transparent_bg.setBrush(QPalette::ColorGroup(i), QPalette::Window, QBrush(color));
            }
            setPalette(transparent_bg);

            setText("<font color='black'>" + text + "</font");
        }
};

} // namespace liquid

#endif // ndef __LIQUID_LABEL_H__

