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

#ifndef __CAUV_F_VIEW_H__
#define __CAUV_F_VIEW_H__

#include <liquid/view.h>

namespace cauv{
namespace gui{

class FView: public liquid::LiquidView {
    Q_OBJECT
    public:
        FView(QWidget *parent = NULL);
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

