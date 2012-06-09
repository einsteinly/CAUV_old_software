/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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
#include <liquid/water/graph.h>
#include <liquid/water/dataSeries.h>

namespace w = liquid::water;

class QMenu;

namespace cauv{

class CauvNode;

namespace gui{
namespace f{

class Manager;
class Menu;

class FView: public liquid::LiquidView{
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

        virtual void mouseMoveEvent(QMouseEvent *event);

    private:
        // types
        typedef boost::shared_ptr<QAction> QAction_ptr;
        typedef QSet<QAction_ptr> QAction_ptr_set;
        struct MenuNode{
            QAction_ptr action;
            QString group_name;
            QList< boost::shared_ptr<MenuNode> > kids;
        };

    private:
        // methods
        static float split(std::string const& word, QAction_ptr_set actions);
        void initMenu();
        void initMenu(MenuNode& parent, QAction_ptr_set actions);
        void _updateOverlays();
        void _buildMenu(cauv::gui::f::Menu* menu, MenuNode const& node);

    private Q_SLOTS:
        // slots
        void menuActioned();

        void postData();

    private:
        // data
        boost::shared_ptr<CauvNode> m_cauv_node;
        boost::shared_ptr<Manager> m_manager;
        MenuNode m_contextmenu_root;

        // +ve coordinates are relative to left and top, -ve coordinates are
        // relative to right and bottom of the view:
        // This is used, for example, to draw buttons inside the view in a
        // consistent position as the view is moved around.
        std::vector< std::pair<QPoint, QGraphicsWidget*> > m_overlay_items;

        // !!! temporary graph dev stuff
        w::DataSeries_ptr m_pct_series;
        w::DataSeries_ptr m_pct2_series;
        w::DataSeries_ptr m_unlim_series;
        w::DataSeries_ptr m_angle_series;
        QTimer* m_redraw_timer;
        QTimer* m_data_timer;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

