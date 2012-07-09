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

#ifndef __CAUV_GUI_F_IMAGE_SOURCE_H__
#define __CAUV_GUI_F_IMAGE_SOURCE_H__

#include <QObject>

#include <boost/shared_ptr.hpp>

namespace cauv{

class GuiImageMessage;

namespace gui{
namespace f{

class ImageSource: public QObject{
        Q_OBJECT
    public:
        void emitImage(boost::shared_ptr<const GuiImageMessage> msg){
            Q_EMIT newImageAvailable(msg);
        }
    Q_SIGNALS:
        void newImageAvailable(boost::shared_ptr<const GuiImageMessage>);
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_IMAGE_SOURCE_H__
