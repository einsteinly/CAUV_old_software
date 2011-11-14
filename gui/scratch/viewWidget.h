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

#ifndef __CAUV_SCRATCH_VIEWWIDGET_H__
#define __CAUV_SCRATCH_VIEWWIDGET_H__

#include <QtGui/QGraphicsView>

#include <boost/shared_ptr.hpp>

namespace cauv{

class CauvNode;

namespace gui{ 
namespace f{

class Manager;

class ViewWidget: public QGraphicsView{
    Q_OBJECT
    public:
        ViewWidget(boost::shared_ptr<CauvNode> node, QWidget *parent = NULL);
        virtual QSize sizeHint() const {return QSize(1280, 800);}

    private:
        boost::shared_ptr<CauvNode> m_cauv_node;
        boost::shared_ptr<Manager> m_manager;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_SCRATCH_VIEWWIDGET_H__

