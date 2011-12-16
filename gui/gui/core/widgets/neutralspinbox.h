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

#ifndef NEUTRALSPINBOX_H
#define NEUTRALSPINBOX_H

#include <QSpinBox>

namespace cauv {
    namespace gui {

        class NeutralSpinBox : public QSpinBox {
            Q_OBJECT

        public:

            Q_PROPERTY(int neutral READ getNeutral WRITE setNeutral USER true)
            Q_PROPERTY(int value READ value WRITE setValue USER true)

            NeutralSpinBox(QWidget * parent = 0);

            int getNeutral() const;
            void setNeutral(int neutral);

        protected:
            int m_neutral;
        };


    } // namespace gui
} // namespace cauv

#endif // NEUTRALSPINBOX_H
