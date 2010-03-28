#ifndef __EDIT_TEXT_RENDERABLE_H__
#define __EDIT_TEXT_RENDERABLE_H__

#include "menu.h"
#include "text.h"

#include <boost/make_shared.hpp>

#include <QKeyEvent>

#include <common/debug.h>

typedef void (*done_func) (std::string);
template<done_func done_F>
class EditText: public Menu{
    public:
        EditText(PipelineWidget& p, std::string const& text, BBox size)
            : Menu(p), m_bbox(size), m_fixed_size(false),
              m_txt_prev(boost::make_shared<Text>(boost::ref(p), text)),
              m_txt_post(boost::make_shared<Text>(boost::ref(p), "")){
            if(m_bbox.xmax > m_bbox.xmin && m_bbox.ymax > m_bbox.ymin)
                m_fixed_size = true;
            else
                error() << "only fixed size version is implemented";
            updateTextPositions();
        }

        virtual ~EditText(){
            done_F(*m_txt_prev + *m_txt_post);
        }
        
        virtual void draw(bool picking){
            glColor4f(1.0, 1.0, 1.0, 0.8);
            glBegin(GL_QUADS);
            glVertex2f(m_bbox.xmin, m_bbox.ymax);
            glVertex2f(m_bbox.xmin, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymax);
            glEnd();

            // keep everything else nicely in front:
            glTranslatef(0, 0, 0.1);
            
            GLdouble lh_clip_plane[4] = {1, 0, 0, -m_bbox.xmin};
            GLdouble rh_clip_plane[4] = {-1, 0, 0, m_bbox.xmax};
            glClipPlane(GL_CLIP_PLANE0, lh_clip_plane);
            glClipPlane(GL_CLIP_PLANE1, rh_clip_plane);
            glEnable(GL_CLIP_PLANE0);
            glEnable(GL_CLIP_PLANE1);

            glPushMatrix();
            glTranslatef(m_txt_prev->m_pos_x, m_txt_prev->m_pos_y, 0);
            m_txt_prev->draw(picking);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(m_txt_prev->m_pos_x + m_txt_prev->bbox().w() + 1 + m_cur_w/2, 0, 0);
            glColor4f(0.8, 0.1, 0.1, 0.9);
            glLineWidth(m_cur_w);
            glBegin(GL_LINES);
            glVertex2f(0, m_bbox.ymin);
            glVertex2f(0, m_bbox.ymax);
            glEnd();
            glPopMatrix();
            
            if(m_txt_prev->bbox().w() + m_cur_w + 1 <= m_bbox.w() &&
               m_txt_post->m_pos_x + m_txt_post->bbox().xmin < m_bbox.xmax){
                //TODO: RH clipping plane
                glPushMatrix();
                glTranslatef(m_txt_post->m_pos_x, m_txt_post->m_pos_y, 0);
                m_txt_post->draw(picking);
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
                BBox r = {0};
                return r;
            }
        }

        virtual bool keyPressEvent(QKeyEvent* event){
            std::string new_text = event->text().toStdString();
            switch(event->key()){
                case Qt::Key_Left:
                    debug() << "EditText: key left";
                    if(m_txt_prev->size())
                        m_txt_post->insert(m_txt_post->begin(), *m_txt_prev->rbegin());
                    // fall through
                case Qt::Key_Backspace:
                    if(m_txt_prev->size())
                        m_txt_prev->erase(m_txt_prev->end()-1);
                    break;
                case Qt::Key_Right:
                    debug() << "EditText: key right";
                    if(m_txt_post->size())
                        *m_txt_prev += (*m_txt_post)[0];
                    // fall through
                case Qt::Key_Delete:
                    if(m_txt_post->size())
                        m_txt_post->erase(m_txt_post->begin());
                    break;
                default:
                    if(new_text.size())
                        *m_txt_prev += new_text;
                    else
                        return false;
            }
            updateTextPositions();
            m_parent.updateGL();
            return true; 
        }

    private:
        void updateTextPositions(){
            m_txt_prev->updateBbox();
            m_txt_post->updateBbox();
            if(m_txt_prev->bbox().w() + m_cur_w + 1 > m_bbox.w()){
                m_txt_prev->m_pos_x = m_bbox.xmax - m_txt_prev->bbox().xmax - m_cur_w - 1;
            }else{
                m_txt_prev->m_pos_x = m_bbox.xmin - m_txt_prev->bbox().xmin;
            }
            m_txt_post->m_pos_x = m_txt_prev->m_pos_x + m_txt_prev->bbox().xmax + m_cur_w + 2;
            m_txt_post->m_pos_y = 0;
            m_txt_prev->m_pos_y = 0;
        }

        BBox m_bbox;
        bool m_fixed_size;
        boost::shared_ptr<Text> m_txt_prev;
        boost::shared_ptr<Text> m_txt_post;

        static const double m_cur_w = 2;
};

#endif // ndef __EDIT_TEXT_RENDERABLE_H__

