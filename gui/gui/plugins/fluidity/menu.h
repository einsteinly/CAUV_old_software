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

#ifndef __CAUV_GUI_F_MENU_H__
#define __CAUV_GUI_F_MENU_H__

#include <algorithm>
#include <cassert>
#include <boost/bind.hpp>

#include <QMenu>
#include <QTextEdit>
#include <QCoreApplication>
#include <QKeyEvent>

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace f{

class Menu: public QMenu{
    Q_OBJECT
    public:
        Menu(QWidget* parent=0)
            : QMenu(parent),
              m_textedit(NULL),
              m_header(NULL),
              m_unfiltered_actions(),
              m_unfiltered_menus(),
              m_parent_menu(NULL){
            m_textedit = new QTextEdit(""/*"Beware, egregious hack!"*/, this);
            m_textedit->setLineWrapMode(QTextEdit::NoWrap);
            m_textedit->setFocus(Qt::PopupFocusReason);
            m_textedit->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            m_textedit->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            //m_textedit->verticalScrollbar()->hide();
            connect(m_textedit, SIGNAL(textChanged()), this, SLOT(updateSearch()));
            // leave space for the search box (has to draw within rect())
            //setTitle doesn't seem to add a title to contextmenus...
            //setTitle(m_header_string);
            m_header = QMenu::addAction("-----------");
            QAction* separator = new QAction(this);
            separator->setSeparator(true);
            QMenu::addAction(separator);
        }

        // Use to construct a submenu (no search box)
        Menu(QString topic, QWidget* parent=0)
            : QMenu(topic, parent),
              m_textedit(NULL),
              m_header(NULL),
              m_unfiltered_actions(),
              m_unfiltered_menus(),
              m_parent_menu(dynamic_cast<Menu*>(parent)){
        }


        void addAction(QAction* action){
            // NB: ownership of action NOT transferred
            QMenu::addAction(action);
            m_unfiltered_actions << action;
        }

        void addMenu(Menu* m){
            QMenu::addMenu(m);
            m_unfiltered_menus << m;
        }

        QAction* exec(QPoint const& p, QAction* action = 0){
            updateTextBoxGeom();
            return QMenu::exec(p, action);
        }

    public Q_SLOTS:
        void updateSearch(){
            QString search = m_textedit->toPlainText();
            updateSearchWithString(search);
        }


    protected:
        virtual void keyPressEvent(QKeyEvent* e){
            static bool recursed_hack = false;
            if(m_parent_menu){
                debug() << "forward keypress to parent menu...";
                m_parent_menu->keyPressEvent(e);
            }else{
                switch(e->key())
                {
                    case Qt::Key_Up:
                        setActiveIndex(m_active_index - 1, false);
                        break;
                    case Qt::Key_Down:
                        setActiveIndex(m_active_index + 1, false);
                        break;
                    case Qt::Key_Enter:
                    case Qt::Key_Return:
                        if (m_active_action)
                            m_active_action->activate(QAction::Trigger);
                        break;
                    case Qt::Key_Escape:
                        close();
                        break;
                    default:
                        assert(m_textedit);
                        if(!recursed_hack){
                            recursed_hack = true;
                            QCoreApplication::sendEvent(
                                m_textedit,
                                new QKeyEvent(
                                    e->type(), e->key(), e->modifiers(), e->text(), e->isAutoRepeat(), e->count()
                                )
                            );
                            recursed_hack = false;
                        }
                        break;
                }

            }
            e->accept();
        }
        
        // make sure the textedit is appropriately resized
        virtual void showEvent(QShowEvent* event){
            updateTextBoxGeom();
            QMenu::showEvent(event);
        }

        int updateSearchWithString(QString search){
            debug(3) << title().toStdString() << "::" << __func__ << search.toStdString();
            
            m_total_filtered = 0;

            QList<QAction*> current_actions = actions();
            for(int i = 0; i < m_unfiltered_actions.size(); i++){
                QAction* a = m_unfiltered_actions.at(i);
                bool is_match = a->text().contains(search, Qt::CaseInsensitive);
                if (is_match) {
                    ++m_total_filtered;
                }

                if(current_actions.contains(a)){
                    if (!is_match && !a->isSeparator() && a != m_header){
                        QMenu::removeAction(a);
                    }
                }else{
                    if(is_match || a->isSeparator()){
                        QList<QAction*>::iterator next_a = std::find_if(
                            m_unfiltered_actions.begin()+i, m_unfiltered_actions.end(),
                            boost::bind(&QList<QAction*>::contains, &current_actions, _1)
                        );
                        if (next_a != m_unfiltered_actions.end())
                            QMenu::insertAction(*next_a, a);
                        else
                            QMenu::addAction(a);
                    }
                }
            }
            // Filter submenus... (the menus actions are included in actions(),
            // but disable them instead of removing/adding)
            foreach(Menu* m, m_unfiltered_menus){
                int subcount = m->updateSearchWithString(search);
                m_total_filtered += subcount;
                QAction* ma = m->menuAction();
                if(subcount > 0){
                    if(!ma->isEnabled()){
                        //ma->setVisible(true);
                        ma->setEnabled(true);
                    } 
                }else if(subcount == 0 && ma->isEnabled()){
                    //ma->setVisible(false);
                    ma->setEnabled(false);
                }
            }

            if (!m_parent_menu)
                setActiveIndex(0);
            updateTextBoxGeom();
            
            debug(3) << title().toStdString() << "::" << __func__ << "total filtered = " << m_total_filtered;

            return m_total_filtered;
        }

        void setActiveIndex(int i, bool clearOnError = true)
        {
            if (m_total_filtered == 0) {
                if (clearOnError) {
                    setActiveAction(NULL);
                    m_active_action = NULL;
                }
                return;
            }

            i = i % m_total_filtered;
            if (i < 0)
                i = m_total_filtered + i;

            int saved_i = i;
            std::vector<std::pair<Menu*, QAction*> > action_stack;
            QAction* action = setActiveIndex(i, action_stack);
            if (action) {
                m_active_index = saved_i;
                m_active_action = action;
            } else {
                if (clearOnError) {
                    setActiveAction(NULL);
                    m_active_action = NULL;
                }
            }
        }
        QAction* setActiveIndex(int& i, std::vector<std::pair<Menu*, QAction*> >& action_stack)
        {
            debug(3) << title().toStdString() << "::" << __func__ << i;

            if (i < 0) {
                return NULL;
            }
            
            // Try each action
            foreach (QAction* a, actions()) {
                if (!a->isEnabled() || a->isSeparator() || a == m_header)
                    continue;
                
                Menu* m = dynamic_cast<Menu*>(a->menu());
                if (m) {
                    // This is a menu, so recurse
                    action_stack.push_back(std::make_pair(this, a));
                    
                    // If the menu successfully set the active action, bubble up
                    // Otherwise, it'll change i, pop itself off the stack, and keep going
                    QAction* ret = m->setActiveIndex(i, action_stack);
                    if (ret)
                        return ret;
                    
                    action_stack.pop_back();
                } else if (i == 0) {
                    // We've found it! Set everything to be active, and escape
                    typedef std::pair<Menu*, QAction*> stack_pair_t;
                    foreach(stack_pair_t p, action_stack) {
                        if (p.first->activeAction() != p.second) {
                            debug(3) << p.first->title().toStdString() << "::setActiveAction" << p.second->text().toStdString();
                            p.first->setActiveAction(p.second);
                        }
                    }
                    setActiveAction(a);
                    return a;
                } else {
                    --i;
                }
            }

            // Sorry Mario, your action is in another castle
            return NULL;
        }
        
        struct HasNonFilteredStuff{
            bool operator()(Menu* m){
                return m->filteredSize();
            }
        };
        int filteredSize() const{
            // this made sense at the time, and it works
            return actions().size() - m_unfiltered_menus.size() +
                std::count_if(
                    m_unfiltered_menus.begin(), m_unfiltered_menus.end(), HasNonFilteredStuff()
                );
        }

    private:
        void updateTextBoxGeom(){
            if(m_textedit)
                m_textedit->setGeometry(2,2,rect().width()-4,24);
        }

    private:
        int m_active_index;
        QAction* m_active_action;
        int m_total_filtered;
        QTextEdit* m_textedit;
        QAction* m_header;
        QList<QAction*> m_unfiltered_actions;
        QList<Menu*> m_unfiltered_menus;
        Menu* m_parent_menu;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_MENU_H__

