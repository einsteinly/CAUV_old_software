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
        }
};

} // namespace liquid

#endif // ndef __LIQUID_LABEL_H__

