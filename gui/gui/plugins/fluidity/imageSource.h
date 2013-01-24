/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
