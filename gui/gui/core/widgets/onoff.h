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

#ifndef __CAUV_ONOFF_H
#define __CAUV_ONOFF_H

#include <QCheckBox>

#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

        class OnOffSlider : public QCheckBox {
            Q_OBJECT

        public:

            Q_PROPERTY(bool checked READ isChecked WRITE setChecked USER true)

            void mouseReleaseEvent(QMouseEvent *e);

            void mousePressEvent(QMouseEvent *);

            OnOffSlider(QWidget * parent = 0);

            void paintEvent(QPaintEvent *);
        };

    } // namespace gui
} // namespace cauv

#endif // __CAUV_ONOFF_H
