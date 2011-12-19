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

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace f{

class Menu: protected QMenu{
    Q_OBJECT
    public:
        Menu(QWidget* parent=0)
            : QMenu(parent),
              m_textedit(NULL),
              m_header_string("----------"),
              m_unfiltered_actions(){
            m_textedit = new QTextEdit(""/*"Beware, egregious hack!"*/, this);
            m_textedit->setLineWrapMode(QTextEdit::NoWrap);
            m_textedit->setFocus(Qt::PopupFocusReason);
            connect(m_textedit, SIGNAL(textChanged()), this, SLOT(updateSearch()));
            // leave space for the search box (has to draw within rect())
            //setTitle doesn't seem to add a title to contextmenus...
            //setTitle(m_header_string); 
            QMenu::addAction(m_header_string);
            m_textedit->setGeometry(0,0,400,22);
        }

        void addAction(QAction* action){
            // NB: ownership of action NOT transferred
            QMenu::addAction(action);
            m_unfiltered_actions << action;
        }

        QAction* exec(QPoint const& p, QAction* action = 0){
            return QMenu::exec(p, action);
        }

    public Q_SLOTS:
        void updateSearch(){
            debug() << __func__ << m_textedit->toPlainText().toStdString();
            QString search = m_textedit->toPlainText();
            QList<QAction*> current_actions = actions();
            for(int i = 0; i < m_unfiltered_actions.size(); i++){
                QAction* a = m_unfiltered_actions.at(i);
                if(current_actions.contains(a)){
                    if(!a->text().contains(search, Qt::CaseInsensitive) &&
                       !a->isSeparator() && 
                       a->text() != m_header_string){
                        QMenu::removeAction(a);
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
                    }
                }
            }
            updateTextBoxGeom();
        }


    private:
        void updateTextBoxGeom(){
            m_textedit->setGeometry(0,0,rect().width(),22);
        }

    private:
        QTextEdit* m_textedit;
        QString m_header_string;
        QList<QAction*> m_unfiltered_actions;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_MENU_H__

