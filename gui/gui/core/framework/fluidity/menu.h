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
              m_header_string("----------"),
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
            QMenu::addAction(m_header_string);
            QAction* separator = new QAction(this);
            separator->setSeparator(true);
            QMenu::addAction(separator);
        }

        // Use to construct a submenu (no search box)
        Menu(QString topic, QWidget* parent=0)
            : QMenu(topic, parent),
              m_textedit(NULL),
              m_header_string(),
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
            }
            e->accept();
        }
        
        // make sure the textedit is appropriately resized
        virtual void showEvent(QShowEvent* event){
            updateTextBoxGeom();
            QMenu::showEvent(event);
        }

        void updateSearchWithString(QString search){
            debug(3) << __func__ << search.toStdString();
            QList<QAction*> current_actions = actions();
            //for(int i = m_unfiltered_actions.size()-1; i >=0; i--){
            QAction* single_active_action = NULL;
            for(int i = 0; i < m_unfiltered_actions.size(); i++){
                QAction* a = m_unfiltered_actions.at(i);
                if(current_actions.contains(a)){
                    if(!a->text().contains(search, Qt::CaseInsensitive) &&
                       !a->isSeparator() &&
                       a->text() != m_header_string){
                        QMenu::removeAction(a);
                    }else{
                        single_active_action = a;
                    }
                }else{
                    if(a->text().contains(search, Qt::CaseInsensitive) ||
                       a->isSeparator()){
                        QAction* next_a = NULL;
                        for(int j = i+1; next_a == NULL && j < m_unfiltered_actions.size(); j++){
                            QAction* maybe_next = m_unfiltered_actions.at(j);
                            if(current_actions.contains(maybe_next))
                                next_a = maybe_next;
                        }
                        QMenu::insertAction(next_a, a);
                        single_active_action = a;                        
                    }
                }
            }
            if(actions().size() - m_unfiltered_menus.size() != 1){
                single_active_action = NULL;
            }
            // Filter submenus... (the menus actions are included in actions(),
            // but disable them instead of removing/adding)
            int num_active_submenus = 0;
            QAction* single_active_submenu_action = NULL;
            foreach(Menu* m, m_unfiltered_menus){
                m->updateSearchWithString(search);
                QAction* ma = m->menuAction();
                if(m->filteredSize()){
                    if(!ma->isEnabled()){
                        //ma->setVisible(true);
                        ma->setEnabled(true);
                    } 
                    num_active_submenus++;
                    single_active_submenu_action = ma;
                }else if((!m->filteredSize()) && ma->isEnabled()){
                    //ma->setVisible(false);
                    ma->setEnabled(false);
                }
            }
            // can't get this to work:
            /*if(single_active_action && num_active_submenus == 0){
                debug() << "set single active action";            
                setActiveAction(single_active_action);
            }else if(num_active_submenus == 1){
                debug() << "set single active submenu";
                setActiveAction(single_active_submenu_action);
            }*/
            updateTextBoxGeom();
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
        QTextEdit* m_textedit;
        QString m_header_string;
        QList<QAction*> m_unfiltered_actions;
        QList<Menu*> m_unfiltered_menus;
        Menu* m_parent_menu;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_MENU_H__

