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

#include <boost/shared_ptr.hpp>

#include <liquid/view.h>

namespace cauv{

class CauvNode;

namespace gui{
namespace f{

class Manager;

class FView: public liquid::LiquidView {
    Q_OBJECT
    public:
        FView(boost::shared_ptr<CauvNode> node, QWidget *parent = NULL);

    protected:
        // QWidget
        void contextMenuEvent(QContextMenuEvent *event);
        //void resizeEvent(QResizeEvent* event);
        //void scrollContentsBy(int dx, int dy);
        void paintEvent(QPaintEvent * event);

        // temporary keyboard shortcut hook:
        virtual void keyPressEvent(QKeyEvent *event);

    private:
        void buildMenus();

    private:
        void _updateOverlays();

        boost::shared_ptr<CauvNode> m_cauv_node;
        boost::shared_ptr<Manager> m_manager;

        QList<QAction*> m_contextmenu_actions;

        // +ve coordinates are relative to left and top, -ve coordinates are
        // relative to right and bottom of the view:
        // This is used, for example, to draw buttons inside the view in a
        // consistent position as the view is moved around.
        std::vector< std::pair<QPoint, QGraphicsWidget*> > m_overlay_items;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

