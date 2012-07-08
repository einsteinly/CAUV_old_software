#ifndef __EDIT_TEXT_RENDERABLE_H__
#define __EDIT_TEXT_RENDERABLE_H__

#include "menu.h"
#include "text.h"

#include <boost/make_shared.hpp>

#include <QtOpenGL>
#include <QClipboard>

#include <vector>        // needed for the undo function

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace pw{

template<typename done_arg_T>
class EditText: public Menu{
    public:
        typedef void (*done_fp) (done_arg_T, std::string const&);
        EditText(container_ptr_t c, std::string const& text, BBox const& size,
                 done_fp done_f, done_arg_T done_f_arg)
            : Menu(c), m_bbox(size), m_fixed_size(false), m_edited(false),
              m_txt_prev(boost::make_shared<Text>(c, text)),
              m_txt_post(boost::make_shared<Text>(c, "")),
              m_done_f(done_f), m_done_f_arg(done_f_arg),
              m_cursor_colour(0.8, 0.1, 0.1, 0.9){
            if(m_bbox.area() > 0)
                m_fixed_size = true;
            else
                error() << "only fixed size version is implemented";
            updateTextPositions();
            undo_position = 0;
            undoPreStorage.push_back(*m_txt_prev);    // sets the first member to whatever is already there
            undoPstStorage.push_back(*m_txt_post);
        }

        virtual ~EditText(){
            if(m_edited)
                m_done_f(m_done_f_arg, *m_txt_prev + *m_txt_post);
        }
        
        virtual void draw(drawtype_e flags){
            // keep everything nicely in front:
            glTranslatef(0, 0, 0.1);

            glColor4f(1.0, 1.0, 1.0, 0.8);
            glBox(m_bbox);

            // and the text even more in front:
            glTranslatef(0, 0, 0.1);
            
            GLdouble lh_clip_plane[4] = {1, 0, 0, -m_bbox.min.x};
            GLdouble rh_clip_plane[4] = {-1, 0, 0, m_bbox.max.x};
            glClipPlane(GL_CLIP_PLANE0, lh_clip_plane);
            glClipPlane(GL_CLIP_PLANE1, rh_clip_plane);
            glEnable(GL_CLIP_PLANE0);
            glEnable(GL_CLIP_PLANE1);

            glPushMatrix();
            glTranslatef(m_txt_prev->m_pos);
            m_txt_prev->draw(flags);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(m_txt_prev->m_pos.x + m_txt_prev->bbox().w() + 1 + m_cur_w/2.0, 0, 0);
            glColor(m_cursor_colour);
            glLineWidth(m_cur_w);
            glBegin(GL_LINES);
            glVertex2f(0, m_bbox.min.y);
            glVertex2f(0, m_bbox.max.y);
            glEnd();
            glPopMatrix();
            
            if(m_txt_prev->bbox().w() + m_cur_w + 1 <= m_bbox.w() &&
               m_txt_post->m_pos.x + m_txt_post->bbox().min.x < m_bbox.max.x){
                //TODO: RH clipping plane
                glPushMatrix();
                glTranslatef(m_txt_post->m_pos);
                m_txt_post->draw(flags);
                glPopMatrix();
            }
            
            glDisable(GL_CLIP_PLANE0);
            glDisable(GL_CLIP_PLANE1);
            
        }

        virtual BBox bbox(){
            if(m_fixed_size)
                return m_bbox;
            else{
                error() << "only fixed size version is implemented";
                return BBox();
            }
        }

        virtual bool keyPressEvent(KeyEvent const& event){
            std::string new_text = event.text().toStdString();
            debug() << BashColour::Green << "editText::keyPressEvent"
                    << std::hex << event.key() << event.modifiers();
            if ( event.modifiers() == Qt::ControlModifier )
            {
                switch(event.key())
                {
                    case Qt::Key_V:
                    {
                        QClipboard *cb = QApplication::clipboard();
                        std::string text = cb->text().toStdString();
                        if ( text.size() )
                        {
                            debug() << "paste text:" << text;
                            // clipboard has something in it
                            *m_txt_prev += text;    // add clipboard text
                            updateUndo();    // updates the undo buffer
                        }
                        else
                            warning() << "no text to paste";
                        break;
                    }
                    case Qt::Key_D:
                        // deletes the string
                        m_txt_prev->clear();
                        m_txt_post->clear();
                        updateUndo();
                        break;
                    case Qt::Key_Z:
                        // undos
                        if ( undo_position )
                            --undo_position; // undo_position != 0
                        if ( undo_position > undoPreStorage.size() -1 )
                            undo_position = undoPreStorage.size() -1;
                        writeUndoRedo();                        
                        break;
                    case Qt::Key_Y:
                        // redoes
                        ++undo_position;
                        if ( undo_position > undoPreStorage.size() -1 )
                            undo_position = undoPreStorage.size() -1;
                        writeUndoRedo();
                        break;
                    default:
                        debug() << "no action on" << event.key() << "(" << new_text << ")";
                        // do fuck all, go work out on your shake weight if you have one
                        break;
                }
            }            
            else
            {
                switch(event.key()){
                    case Qt::Key_Enter:
                    case Qt::Key_Return:
                        debug() << "EditText: key enter";
                        m_context->removeMenu(shared_from_this());
                        // this has now probably been destroyed, must return pronto!
                        return true;
                    case Qt::Key_Left:
                        debug(2) << "EditText: key left";
                        if(m_txt_prev->size())
                            m_txt_post->insert(m_txt_post->begin(), *m_txt_prev->rbegin());
                        // fall through
                    case Qt::Key_Backspace:
                        if(m_txt_prev->size())
                        {
                            m_txt_prev->erase(m_txt_prev->end()-1);
                            updateUndo();    // undates the undo
                        }
                        break;
                    case Qt::Key_Right:
                        debug(2) << "EditText: key right";
                        if(m_txt_post->size())
                            *m_txt_prev += (*m_txt_post)[0];
                        // fall through
                    case Qt::Key_Delete:
                        if(m_txt_post->size())
                        {
                            m_txt_post->erase(m_txt_post->begin());
                            updateUndo();    // undates the undo
                        }
                        break;
                    default:
                        if(new_text.size())
                        {
                            *m_txt_prev += new_text;
                            updateUndo();    // undates the undo
                        }
                        else
                            return false;
                }
            }
            // TODO: be more conservative about m_edited
            m_edited = true;
            updateTextPositions();
            m_context->postRedraw(0);
            return true; 
        }

    private:
        void updateTextPositions(){
            m_txt_prev->updateBbox();
            m_txt_post->updateBbox();
            if(m_txt_prev->bbox().w() + m_cur_w + 1 > m_bbox.w()){
                m_txt_prev->m_pos.x = m_bbox.max.x - m_txt_prev->bbox().max.x - m_cur_w - 1;
            }else{
                m_txt_prev->m_pos.x = m_bbox.min.x - m_txt_prev->bbox().min.x;
            }
            m_txt_post->m_pos.x = m_txt_prev->m_pos.x + m_txt_prev->bbox().max.x + m_cur_w + 2;
            m_txt_post->m_pos.y = 0;
            m_txt_prev->m_pos.y = 0;
        }

        BBox m_bbox;
        bool m_fixed_size;
        bool m_edited;
        boost::shared_ptr<Text> m_txt_prev;
        boost::shared_ptr<Text> m_txt_post;
        
        void updateUndo(  )
        {
            // this updates the undo storage
            // needs to check what position is active and deletes above it
            std::string pre = *m_txt_prev;
            std::string post = *m_txt_post;
            while ( undo_position + 1 != undoPreStorage.size() )
            {
                undoPreStorage.pop_back();    // deal with it James, it aint pretty, but it works
                undoPstStorage.pop_back();
            }
            undoPreStorage.push_back(pre);    
            undoPstStorage.push_back(post);
            undo_position = undoPreStorage.size() -1;    // sets it at the end
        }
        
        void writeUndoRedo()
        {
            // this is called when undo or redo has been pressed and the array number corrected
            m_txt_prev->clear();
            m_txt_post->clear();
            *m_txt_prev += undoPreStorage[undo_position];
            *m_txt_post += undoPstStorage[undo_position];
        }
        
        std::vector<std::string> undoPreStorage;
        std::vector<std::string> undoPstStorage;
        unsigned int undo_position;
        
        done_fp m_done_f;
        done_arg_T m_done_f_arg;

        Colour m_cursor_colour;

        static const int m_cur_w = 2;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __EDIT_TEXT_RENDERABLE_H__

