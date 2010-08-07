#include "OverKey.h"

#include <QtOpenGL>

#include <debug/cauv_debug.h>
#include "util.h"

using namespace pw;
using namespace pw::ok;

Action::f_t Action::null_f = Action::f_t();

Action::Action(f_t const& on_press_f,
               f_t const& on_release_f,
               std::string const& descr,
               renderable_ptr_t decal)
    : on_press_f(on_press_f),
      on_release_f(on_release_f),
      m_descr(descr),
      m_decal(decal){
}

void Action::onPress() const{
    debug() << "Action \"" << m_descr << "\" onPress";
    if(on_press_f)
        on_press_f();
}

void Action::onRelease() const{
    debug() << "Action \"" << m_descr << "\" onRelease";
    if(on_release_f)
        on_release_f();
}


Key::Key(container_ptr_t c, double const& w, double const& h, textmap_t const& text)
    : Renderable(c), m_text(text), m_box(-w/2, -h/2, w/2, h/2){
}

Key::~Key(){
}

void Key::draw(drawtype_e::e){
    glColor4f(1.0, 1.0, 1.0, 0.2);
    glBox(m_box, m_box.h()/5);
}

void Key::draw(Qt::KeyboardModifiers const& mods){
    glBox(m_box);
    if(m_text.count(mods))
        m_text[mods]->draw(drawtype_e::no_flags);
}

BBox Key::bbox(){
    return m_box;
}


bool KeyBind::operator<(KeyBind const& r) const{
    if(int(keycode) < int(r.keycode))
        return true;
    else if (int(keycode) == int(r.keycode))
        return int(modifiers) < int(r.modifiers);
    else
        return false; 
}


/* keymap init function:
 * TODO: detect the keyboard layout (Qt, OS?) and load from file
 */
OverKey::layout_map_t appleEnGBKeys(){
    OverKey::layout_map_t r;

    return r;
}


OverKey::OverKey(container_ptr_t container)
    : Renderable(container), m_layout(), m_actions(){
    m_layout = appleEnGBKeys();
}

bool OverKey::keyPressEvent(QKeyEvent *event){
    KeyBind b = {event->key(), event->modifiers()};
    if(m_actions.count(b)){
        m_actions[b]->onPress();
        return true;
    }
    return false;
}

bool OverKey::keyReleaseEvent(QKeyEvent *event){
    KeyBind b = {event->key(), event->modifiers()};
    if(m_actions.count(b)){
        m_actions[b]->onRelease();
        return true;
    }
    return false;
}

void OverKey::registerKey(KeyBind const& binding, action_ptr_t act){
    if(m_actions.count(binding))
        warning() << "replacing existing binding for binding"
                  << binding.keycode << binding.modifiers;
    m_actions[binding] = act;
}

void OverKey::registerKey(keycode_t const& key, modifiers_t const& mods,
                          action_ptr_t act){
    KeyBind binding = {key, mods};
    registerKey(binding, act);
}

void OverKey::draw(drawtype_e::e flags){
    if(flags & drawtype_e::picking)
        return;
}

