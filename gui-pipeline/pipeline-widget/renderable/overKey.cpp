#include "OverKey.h"

#include <QtOpenGL>

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include "util.h"
#include "text.h"

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


const static Colour BG_Colours[keystate_e::num_values] = {
    Colour(1, 1, 1, 0.2), // released
    Colour(1, 1, 1, 0.4)  // pressed
};
const static Colour Text_Colours[keystate_e::num_values] = {
    Colour(0, 0, 0, 0.3),
    Colour(0, 0, 0, 1.0)
};

Key::Key(container_ptr_t c, keycode_t const& kc, BBox const& size, textmap_t const& text)
    : Renderable(c), m_state(keystate_e::released), m_keycode(kc), m_text(text), m_box(size){
}

Key::Key(container_ptr_t c, keycode_t const& kc, BBox const& size,
         Qt::KeyboardModifiers m1, std::string const& t1,
         Qt::KeyboardModifiers m2, std::string const& t2,
         Qt::KeyboardModifiers m3, std::string const& t3,
         Qt::KeyboardModifiers m4, std::string const& t4)
    : Renderable(c), m_state(keystate_e::released), m_keycode(kc), m_text(), m_box(size){
    m_text[m1] = boost::make_shared<Text>(c, t1, "LiberationSans-Regular.ttf", 12);
    if(m2) m_text[m2] = boost::make_shared<Text>(c, t2, "LiberationSans-Regular.ttf", 12);
    if(m3) m_text[m3] = boost::make_shared<Text>(c, t3, "LiberationSans-Regular.ttf", 12);
    if(m4) m_text[m4] = boost::make_shared<Text>(c, t4, "LiberationSans-Regular.ttf", 12);
    
    centerText();
}

Key::~Key(){
}

void Key::draw(drawtype_e::e){
    glColor(BG_Colours[m_state]);
    glBox(m_box, m_box.h()/8);
}

void Key::draw(Qt::KeyboardModifiers const& mods){
    draw(drawtype_e::no_flags);
    if(m_text.count(mods)){
        glTranslatef(m_text[mods]->m_pos);
        //glColor(Colour(1, 0.2));
        //glBox(m_text[mods]->bbox());
        glColor(Text_Colours[m_state]);
        m_text[mods]->draw(drawtype_e::no_flags);
    }
}

BBox Key::bbox(){
    return m_box;
}

keycode_t const& Key::keyCode() const{
    return m_keycode;
}

void Key::state(keystate_e::e s){
    m_state = s;
}

void Key::centerText(){
    foreach(textmap_t::value_type v, m_text){
        v.second->m_pos = m_pos + Point((bbox().w()/2 - v.second->bbox().w()/2) + v.second->bbox().min.x,
                                        -bbox().h()/2);
        debug() << "text position:" << v.second->m_pos.x << v.second->m_pos.y;
    }
}


bool KeyBind::operator<(KeyBind const& r) const{
    if(int(keycode) < int(r.keycode))
        return true;
    else if (int(keycode) == int(r.keycode))
        return int(modifiers) < int(r.modifiers);
    else
        return false; 
}


class ReturnKey: public Key{
    public:
        ReturnKey(container_ptr_t c, keycode_t const& kc, BBox const& top_size,
                  float step = 2, float descend = 8)
            : Key(c, kc, top_size | (top_size - Point(0, descend))),
              m_top(top_size), m_step(step), m_descend(descend){
        }
        
        virtual void draw(drawtype_e::e flags){
            if(flags & drawtype_e::picking)
                return;
            // TODO: fix this
            glColor(BG_Colours[m_state]);
            /*
             *        *-------------*
             *        | 2222221111  |
             *        |422222211113 |
             *        | 22222211113 |
             *        *----* 511113 |  -
             *             | 511113 |  |
             *             | 511113 |  | descend
             *             |  1111  |  |
             *             *--------*  - 
             *   step |----|
             */
            const float corner = m_top.h() / 8;
            // 1
            glBox(BBox(m_top.min.x + m_step + corner, m_top.min.y - m_descend,
                       m_top.max.x - corner         , m_top.max.y));
            // 2
            glBox(BBox(m_top.min.x + corner         , m_top.min.y,
                       m_top.min.x + m_step + corner, m_top.max.y));
            // 3
            glBox(BBox(m_top.max.x - corner, m_top.min.y + corner - m_descend,
                       m_top.max.x         , m_top.max.y - corner));
            // 4
            glBox(BBox(m_top.min.x         , m_top.min.y + corner,
                       m_top.min.x + corner, m_top.max.y - corner));
            // 5
            glBox(BBox(m_top.min.x + m_step         , m_top.min.y + corner - m_descend,
                       m_top.min.x + m_step + corner, m_top.min.y));
        }
        virtual void draw(Qt::KeyboardModifiers const& /*mods*/){
            draw(drawtype_e::no_flags);
        }
        
    private:
        BBox m_top;
        float m_step;
        float m_descend;
};

const float key_w = 48;
const float key_h = 48;
const float key_p = 3;

/* keymap init function:
 * TODO: detect the keyboard layout (Qt, OS?) and load from file
 */
OverKey::layout_map_t appleEnGBKeys(container_ptr_t c){
    OverKey::layout_map_t r;
    Qt::KeyboardModifiers none = Qt::NoModifier;
    Qt::KeyboardModifiers shift = Qt::ShiftModifier;
    Qt::KeyboardModifiers alt = Qt::AltModifier;
    Qt::KeyboardModifiers num = Qt::KeypadModifier;
    
    const BBox b(0, -key_h, key_w, 0);
    const BBox s(0, -key_h/2, key_w, 0);
    const BBox esc_box(0, -key_h/2, key_w + key_w/4, 0);
    const BBox bksp_box(0, -key_h, key_w + key_w/2, 0);
    const BBox cps_box(0, -key_h, key_w + 3*key_w/4, 0);
    const BBox lshift_box(0, -key_h, key_w + key_w/4, 0);
    const BBox rshift_box(0, -key_h, 2*key_w + key_w/4 + key_p, 0);
    const BBox fn_box = b;//(0, -key_h, key_w*3.0/4, 0);
    const BBox space_box(0, -key_h, key_w*5 + key_p*4, 0);
    const float enter_step = key_w/4;
    const float enter_descend = key_h + key_p;
    
    Point pos(0, 0);

    const key_ptr_t top_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Escape, esc_box, none, "esc"),
        boost::make_shared<Key>(c, Qt::Key_F1,  s, none, "F1"),
        boost::make_shared<Key>(c, Qt::Key_F2,  s, none, "F2"),
        boost::make_shared<Key>(c, Qt::Key_F3,  s, none, "F3"),
        boost::make_shared<Key>(c, Qt::Key_F4,  s, none, "F4"),
        boost::make_shared<Key>(c, Qt::Key_F5,  s, none, "F5"),
        boost::make_shared<Key>(c, Qt::Key_F6,  s, none, "F6"),
        boost::make_shared<Key>(c, Qt::Key_F7,  s, none, "F7"),
        boost::make_shared<Key>(c, Qt::Key_F8,  s, none, "F8"),
        boost::make_shared<Key>(c, Qt::Key_F9,  s, none, "F9"),
        boost::make_shared<Key>(c, Qt::Key_F10, s, none, "F10"),
        boost::make_shared<Key>(c, Qt::Key_F11, s, none, "F11"),
        boost::make_shared<Key>(c, Qt::Key_F12, s, none, "F12"),
        boost::make_shared<Key>(c, Qt::Key_Eject, esc_box, none, "")
    };
    for(unsigned i = 0; i < sizeof(top_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = top_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k)); 
    }
    
    const key_ptr_t num_row[] = {
        boost::make_shared<Key>(c, Qt::Key_section, b, none, "",  shift, "±"),
        boost::make_shared<Key>(c, Qt::Key_1,       b, none, "1", shift, "!"),
        boost::make_shared<Key>(c, Qt::Key_2,       b, none, "2", shift, "@"),
        boost::make_shared<Key>(c, Qt::Key_3,       b, none, "3", shift, ""),
        boost::make_shared<Key>(c, Qt::Key_4,       b, none, "4", shift, "$"),
        boost::make_shared<Key>(c, Qt::Key_5,       b, none, "5", shift, "%"),
        boost::make_shared<Key>(c, Qt::Key_6,       b, none, "6", shift, "^"),
        boost::make_shared<Key>(c, Qt::Key_7,       b, none, "7", shift, "&", num, "7"),
        boost::make_shared<Key>(c, Qt::Key_8,       b, none, "8", shift, "*", num, "8"),
        boost::make_shared<Key>(c, Qt::Key_9,       b, none, "9", shift, "(", num, "9"),
        boost::make_shared<Key>(c, Qt::Key_0,       b, none, "0", shift, ")", num, "/"),
        boost::make_shared<Key>(c, Qt::Key_Minus,   b, none, "-", shift, "_", num, "="),
        boost::make_shared<Key>(c, Qt::Key_Equal,   b, none, "=", shift, "+"),
        boost::make_shared<Key>(c, Qt::Key_Backspace, bksp_box, none, "")
    };
    pos.x = 0;
    pos.y -= key_h/2 + key_p;
    for(unsigned i = 0; i < sizeof(num_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = num_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k)); 
    }

    const key_ptr_t qw_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Tab, bksp_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_Q,   b, none, "q", shift, "Q"),
        boost::make_shared<Key>(c, Qt::Key_W,   b, none, "w", shift, "W"),
        boost::make_shared<Key>(c, Qt::Key_E,   b, none, "e", shift, "E"),
        boost::make_shared<Key>(c, Qt::Key_R,   b, none, "r", shift, "R"),
        boost::make_shared<Key>(c, Qt::Key_T,   b, none, "t", shift, "T"),
        boost::make_shared<Key>(c, Qt::Key_Y,   b, none, "y", shift, "Y"),
        boost::make_shared<Key>(c, Qt::Key_U,   b, none, "u", shift, "U"),
        boost::make_shared<Key>(c, Qt::Key_I,   b, none, "i", shift, "I"),
        boost::make_shared<Key>(c, Qt::Key_O,   b, none, "o", shift, "O"),
        boost::make_shared<Key>(c, Qt::Key_P,   b, none, "p", shift, "P"),
        boost::make_shared<Key>(c, Qt::Key_BraceLeft,  b, none, "[", shift, "{"),
        boost::make_shared<Key>(c, Qt::Key_BraceRight, b, none, "]", shift, "}"),
        boost::make_shared<ReturnKey>(c, Qt::Key_Return,  b, enter_step, enter_descend)
    };
    pos.x = 0;
    pos.y -= key_h + key_p;
    for(unsigned i = 0; i < sizeof(qw_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = qw_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k)); 
    }

    const key_ptr_t as_row[] = {
        boost::make_shared<Key>(c, Qt::Key_CapsLock, cps_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_A, b, none, "a", shift, "A"),
        boost::make_shared<Key>(c, Qt::Key_S, b, none, "s", shift, "S"),
        boost::make_shared<Key>(c, Qt::Key_D, b, none, "d", shift, "D"),
        boost::make_shared<Key>(c, Qt::Key_F, b, none, "f", shift, "F"),
        boost::make_shared<Key>(c, Qt::Key_G, b, none, "g", shift, "G"),
        boost::make_shared<Key>(c, Qt::Key_H, b, none, "h", shift, "H"),
        boost::make_shared<Key>(c, Qt::Key_J, b, none, "j", shift, "J"),
        boost::make_shared<Key>(c, Qt::Key_K, b, none, "k", shift, "K"),
        boost::make_shared<Key>(c, Qt::Key_L, b, none, "l", shift, "L"),
        boost::make_shared<Key>(c, Qt::Key_Semicolon,  b, none, ";", shift, ":"),
        boost::make_shared<Key>(c, Qt::Key_Apostrophe, b, none, "'", shift, "\""),
        boost::make_shared<Key>(c, Qt::Key_Backslash,  b, none, "\\", shift, "|")
    };
    pos.x = 0;
    pos.y -= key_h + key_p;
    for(unsigned i = 0; i < sizeof(as_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = as_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k));
    }

    const key_ptr_t zx_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Shift,   lshift_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_AsciiTilde, b, none, "`", shift, "~"), //? 
        boost::make_shared<Key>(c, Qt::Key_Z,       b, none, "z", shift, "Z"),
        boost::make_shared<Key>(c, Qt::Key_X,       b, none, "x", shift, "X"),
        boost::make_shared<Key>(c, Qt::Key_C,       b, none, "c", shift, "C"),
        boost::make_shared<Key>(c, Qt::Key_V,       b, none, "v", shift, "V"),
        boost::make_shared<Key>(c, Qt::Key_B,       b, none, "b", shift, "B"),
        boost::make_shared<Key>(c, Qt::Key_N,       b, none, "n", shift, "N"),
        boost::make_shared<Key>(c, Qt::Key_M,       b, none, "m", shift, "M"),
        boost::make_shared<Key>(c, Qt::Key_Comma,   b, none, ",", shift, "<"),
        boost::make_shared<Key>(c, Qt::Key_Period,  b, none, ".", shift, ">"),
        boost::make_shared<Key>(c, Qt::Key_Slash,   b, none, "/", shift, "?"),
        boost::make_shared<Key>(c, Qt::Key_Shift,   rshift_box, none, "")
    };
    pos.x = 0;
    pos.y -= key_h + key_p;
    for(unsigned i = 0; i < sizeof(zx_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = zx_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k)); 
    }
    
    const key_ptr_t fn_row[] = {
        boost::make_shared<Key>(c, Qt::Key_NumLock, fn_box, none, ""), // !!!
        boost::make_shared<Key>(c, Qt::Key_Meta,    b, none, "ctrl"),
        boost::make_shared<Key>(c, Qt::Key_Alt,     b, none, "opt"),
        boost::make_shared<Key>(c, Qt::Key_Control, lshift_box, none, "cmd"),
        boost::make_shared<Key>(c, Qt::Key_Space,   space_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_Control, lshift_box, none, "cmd"),
        boost::make_shared<Key>(c, Qt::Key_Enter,   b, none, "")
    };
    pos.x = 0;
    pos.y -= key_h + key_p;
    for(unsigned i = 0; i < sizeof(fn_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = fn_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + key_p;
        if(r.count(k->keyCode())) warning() << "duplicate keycode" << k->keyCode();
        r.insert(std::make_pair(k->keyCode(), k)); 
    }

    key_ptr_t lk = boost::make_shared<Key>(c, Qt::Key_Left,    s, none, "");
    key_ptr_t uk = boost::make_shared<Key>(c, Qt::Key_Up,      s, none, "");
    key_ptr_t dk = boost::make_shared<Key>(c, Qt::Key_Down,    s, none, "");
    key_ptr_t rk = boost::make_shared<Key>(c, Qt::Key_Right,   s, none, "");

    pos.y -= key_h/2;
    lk->m_pos = pos;
    pos.x += key_w + key_p;
    dk->m_pos = pos;
    uk->m_pos = pos + Point(0, key_h/2 + key_p/2);
    pos.x += key_w + key_p;
    rk->m_pos = pos;

    r.insert(std::make_pair(lk->keyCode(), lk));
    r.insert(std::make_pair(uk->keyCode(), uk));
    r.insert(std::make_pair(dk->keyCode(), dk));
    r.insert(std::make_pair(rk->keyCode(), rk));

    return r;
}


OverKey::OverKey(container_ptr_t parent)
    : Renderable(parent), Container(), m_layout(), m_actions(){
    m_layout = appleEnGBKeys(this);
}


Point OverKey::referUp(Point const& p) const{
    return m_context->referUp(p + m_pos);
}

void OverKey::postRedraw(){
    m_context->postRedraw();
}

void OverKey::postMenu(menu_ptr_t m, Point const& tlp, bool p){
    m_context->postMenu(m, tlp, p);
}

void OverKey::removeMenu(menu_ptr_t m){
    m_context->removeMenu(m);
}

void OverKey::remove(renderable_ptr_t){
    error() << __func__ << __LINE__ << "unimplemented";
}

bool OverKey::keyPressEvent(QKeyEvent *event){
    KeyBind b = {event->key(), event->modifiers()};

    m_current_modifiers = b.modifiers;
    layout_map_t::iterator i;
    std::pair<layout_map_t::iterator, layout_map_t::iterator> eq = m_layout.equal_range(b.keycode);
    for(i = eq.first; i != eq.second; i++)
        i->second->state(keystate_e::pressed);
    if(eq.first != eq.second)
        postRedraw();

    if(m_actions.count(b)){
        m_actions[b]->onPress();
        return true;
    }
    return false;
}

bool OverKey::keyReleaseEvent(QKeyEvent *event){
    KeyBind b = {event->key(), event->modifiers()};

    m_current_modifiers = b.modifiers;
    layout_map_t::iterator i;
    std::pair<layout_map_t::iterator, layout_map_t::iterator> eq = m_layout.equal_range(b.keycode);
    for(i = eq.first; i != eq.second; i++)
        i->second->state(keystate_e::released);
    if(eq.first != eq.second)
        postRedraw();

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
    
    typedef std::pair<keycode_t, key_ptr_t> layout_value_t;
    foreach(layout_value_t r, m_layout){
        glPushMatrix();
        glTranslatef(r.second->m_pos);
        r.second->draw(m_current_modifiers);
        glPopMatrix();
    }
}

