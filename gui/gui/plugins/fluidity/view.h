/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_F_VIEW_H__
#define __CAUV_F_VIEW_H__

#include <boost/shared_ptr.hpp>

#include <liquid/view.h>
//#include <liquid/water/graph.h>
//#include <liquid/water/dataSeries.h>

//namespace w = liquid::water;

class QMenu;

namespace cauv{

class CauvNode;

namespace gui{

class Node;
class NodeScene;

namespace f{

class Manager;
class Menu;

class FView: public liquid::LiquidView{
    Q_OBJECT
    public:
        FView(const std::string& pipeline_name,
              boost::shared_ptr<Node> model_parent,
              NodeScene* s,
              boost::shared_ptr<Manager> m,
              QWidget *parent = NULL);
        
        FView(const std::string& pipeline_name,
              boost::shared_ptr<Node> model_parent,
              QWidget *parent = NULL);

        void init(const std::string& pipeline_name,
                  boost::shared_ptr<Node> model_parent,
                  NodeScene* s,
                  boost::shared_ptr<Manager> m,
                  QWidget* parent);

        ~FView();

        enum Mode {TopLevel, Internal};
        void setMode(Mode const& mode);

        boost::shared_ptr<Manager> manager() { return m_manager; }

        NodeScene* scene();

        Q_SIGNALS:
        void closeRequested();

    protected:
        // QWidget
        void contextMenuEvent(QContextMenuEvent *event);
        void paintEvent(QPaintEvent * event);

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
        //static float split(const std::string& word, QAction_ptr_set actions);
        void initMenu();
        //void initMenu(MenuNode& parent, QAction_ptr_set actions);
        void _updateOverlays();
        void _buildMenu(cauv::gui::f::Menu* menu, MenuNode const& node);

    private Q_SLOTS:
        // slots
        void menuActioned();
        void setSceneRectToContents();

#ifdef QT_PROFILE_GRAPHICSSCENE
        void dumpProfile();
#endif // def QT_PROFILE_GRAPHICSSCENE

        //void postData();

    private:
        // methods
        void _initInMode(Mode const& m);

        // data
        boost::shared_ptr<Manager> m_manager;
        MenuNode m_contextmenu_root;

        // +ve coordinates are relative to left and top, -ve coordinates are
        // relative to right and bottom of the view:
        // This is used, for example, to draw buttons inside the view in a
        // consistent position as the view is moved around.
        // !!! FIXME: can use drawForeground overload to do this more nicely
        std::vector< std::pair<QPoint, QGraphicsWidget*> > m_overlay_items;
        
        /*
        // !!! temporary graph dev stuff
        w::DataSeries_ptr m_pct_series;
        w::DataSeries_ptr m_pct2_series;
        w::DataSeries_ptr m_unlim_series;
        w::DataSeries_ptr m_angle_series;
        QTimer* m_data_timer;
        */

        QTimer* m_scenerect_update_timer;
        Mode m_mode;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

