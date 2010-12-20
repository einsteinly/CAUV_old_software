#include "overKey.h"

#include <limits>

#include <QtOpenGL>

#include <boost/make_shared.hpp>
// TODO <utilty/time.h>
#include <boost/date_time.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/bind.hpp>

#include <utility/rounding.h>
#include <utility/string.h>
#include <debug/cauv_debug.h>

#include "../util.h"
#include "text.h"

using namespace cauv::pw;
using namespace cauv::pw::ok;

// Behaviour Defining Constants:
const static float Typing_Max_Delta = 0.3;
const static float Release_Delay = 0.5;

// Layout Constants, etc:

const static int Key_Font_Size = 12;
const static int Corner_Segments = 8;
const static float Key_W = 48;
const static float Key_H = 48;
const static float Key_P = 3;
const static float BG_Border = 32;

const static Colour OK_BG_Colour(0, 0.8);
const static Colour Key_BG_Colours[keystate_e::num_values] = {
    Colour(1, 0.3), // released
    Colour(1, 0.9)  // pressed
};
const static Colour Key_Text_Colours[keystate_e::num_values] = {
    Colour(0.1, 0.6),
    Colour(0.1, 1.0)
};
const static Colour Key_Decal_Colours[keystate_e::num_values] = {
    Colour(0, 0.9),
    Colour(0, 1.0)
};

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

void Action::drawDecal(BBox const& area){
    if(m_decal){
        float scale = 1.0f;
        BBox const& db = m_decal->bbox();
        if(db.area()){
            if(area.w() / db.w() < scale)
                scale = area.w() / db.w();
            if(area.h() / db.h() < scale)
                scale = area.h() / db.h();
        }
        glPushMatrix();
        glTranslatef(area.c());
        glScalef(scale, scale, 1.0f);
        glTranslatef(-db.c());
        m_decal->draw(drawtype_e::no_flags);
        glPopMatrix();
    }
}


Key::Key(container_ptr_t c, keycode_t const& kc1, keycode_t const& kc2, BBox const& size,
         textmap_t const& text)
    : Renderable(c), m_state(keystate_e::released), m_keycodes(), m_text(text), m_box(size){

    if(kc1) m_keycodes.push_back(kc1);
    if(kc2) m_keycodes.push_back(kc2);
}

Key::Key(container_ptr_t c, keycode_t const& kc1, keycode_t const& kc2, BBox const& size,
         Qt::KeyboardModifiers m1, std::string const& t1,
         Qt::KeyboardModifiers m2, std::string const& t2,
         Qt::KeyboardModifiers m3, std::string const& t3,
         Qt::KeyboardModifiers m4, std::string const& t4)
    : Renderable(c), m_state(keystate_e::released), m_keycodes(), m_text(), m_box(size){

    if(kc1) m_keycodes.push_back(kc1);
    if(kc2) m_keycodes.push_back(kc2);

    m_text[m1] = boost::make_shared<Text>(c, t1, "sans", Key_Font_Size);
    if(m2) m_text[m2] = boost::make_shared<Text>(c, t2, "sans", Key_Font_Size);
    if(m3) m_text[m3] = boost::make_shared<Text>(c, t3, "sans", Key_Font_Size);
    if(m4) m_text[m4] = boost::make_shared<Text>(c, t4, "sans", Key_Font_Size);

    centerText();
}

Key::~Key(){
}

void Key::draw(drawtype_e::e){ 
}

void Key::draw(Qt::KeyboardModifiers const& mods, Colour const& mul){
    glColor(Key_BG_Colours[m_state] * mul);
    glBox(m_box, m_box.h()/8);

    if(m_text.count(mods)){
        glTranslatef(m_text[mods]->m_pos);
        //glColor(Colour(1, 0.2));
        //glBox(m_text[mods]->bbox());
        glColor(Key_Text_Colours[m_state] * mul);
        m_text[mods]->draw(drawtype_e::no_flags);
    }
}

BBox Key::bbox(){
    return m_box;
}

std::vector<keycode_t> const& Key::keyCodes() const{
    return m_keycodes;
}

void Key::state(keystate_e::e s){
    if(m_state != s){
        m_state = s;
        m_context->postRedraw(0);
    }
}

keystate_e::e Key::state() const{
    return m_state;
}

void Key::centerText(){
    foreach(textmap_t::value_type v, m_text){
        v.second->m_pos = m_pos + Point((bbox().w()/2 - v.second->bbox().w()/2) + v.second->bbox().min.x,
                                        -bbox().h()/2);
        debug(9) << "text position:" << v.second->m_pos.x << v.second->m_pos.y;
    }
}

KeyBind::KeyBind(keycode_t const& kc, modifiers_t const& m)
    : keycode(kc), modifiers(m){
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
        ReturnKey(container_ptr_t c, keycode_t const& kc, keycode_t const& kc2,
                  BBox const& top_size, float step = 2, float descend = 8)
            : Key(c, kc, kc2, top_size | (top_size - Point(0, descend))),
              m_top(top_size), m_step(step), m_descend(descend){
        }
        
        virtual void draw(Qt::KeyboardModifiers const& /*mods*/, Colour const& mul){
            glColor(Key_BG_Colours[m_state] * mul);
            /*
             *        *-------------*
             *        | A222211111B |
             *        | 42222111113 |
             *        | E2222111113 |
             *        *----* 511113 |  -
             *             | 511113 |  |
             *             | 511113 |  | descend
             *             | D1111C |  |
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
            // A
            glTranslatef(corner, -corner, 0);
            glSegment(corner, -90, 0, Corner_Segments);
            // B
            glTranslatef(m_top.w() - 2*corner, 0, 0);
            glSegment(corner, 0, 90, Corner_Segments);
            // C
            glTranslatef(0, 2*corner - (m_descend+m_top.h()), 0);
            glSegment(corner, 90, 180, Corner_Segments);
            // D
            glTranslatef(m_step + 2*corner - m_top.w(), 0, 0);
            glSegment(corner, -180, -90, Corner_Segments);
            // E
            glTranslatef(-m_step, m_descend, 0);
            glSegment(corner, -180, -90, Corner_Segments);

        }
        
    private:
        BBox m_top;
        float m_step;
        float m_descend;
};


/* keymap init function:
 * TODO: detect the keyboard layout (Qt, OS?) and load from file
 */
OverKey::layout_map_t appleEnGBKeys(container_ptr_t c){
    OverKey::layout_map_t r;
    Qt::KeyboardModifiers none = Qt::NoModifier;
    Qt::KeyboardModifiers shift = Qt::ShiftModifier;
    Qt::KeyboardModifiers alt = Qt::AltModifier;
    Qt::KeyboardModifiers num = Qt::KeypadModifier;
    
    const BBox b(0, -Key_H, Key_W, 0);
    const BBox s(0, -Key_H/2, Key_W, 0);
    const BBox esc_box(0, -Key_H/2, Key_W + Key_W/4, 0);
    const BBox bksp_box(0, -Key_H, Key_W + Key_W/2, 0);
    const BBox cps_box(0, -Key_H, Key_W + 3*Key_W/4, 0);
    const BBox lshift_box(0, -Key_H, Key_W + Key_W/4, 0);
    const BBox rshift_box(0, -Key_H, 2*Key_W + Key_W/4 + Key_P, 0);
    const BBox fn_box = b;//(0, -Key_H, Key_W*3.0/4, 0);
    const BBox space_box(0, -Key_H, Key_W*5 + Key_P*4, 0);
    const float enter_step = Key_W/4;
    const float enter_descend = Key_H + Key_P;
    
    Point pos(0, 0);

    const key_ptr_t top_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Escape, 0, esc_box, none, "esc"),
        boost::make_shared<Key>(c, Qt::Key_F1, 0,  s, none, "F1"),
        boost::make_shared<Key>(c, Qt::Key_F2, 0,  s, none, "F2"),
        boost::make_shared<Key>(c, Qt::Key_F3, 0,  s, none, "F3"),
        boost::make_shared<Key>(c, Qt::Key_F4, 0,  s, none, "F4"),
        boost::make_shared<Key>(c, Qt::Key_F5, 0,  s, none, "F5"),
        boost::make_shared<Key>(c, Qt::Key_F6, 0,  s, none, "F6"),
        boost::make_shared<Key>(c, Qt::Key_F7, 0,  s, none, "F7"),
        boost::make_shared<Key>(c, Qt::Key_F8, 0,  s, none, "F8"),
        boost::make_shared<Key>(c, Qt::Key_F9, 0,  s, none, "F9"),
        boost::make_shared<Key>(c, Qt::Key_F10, 0, s, none, "F10"),
        boost::make_shared<Key>(c, Qt::Key_F11, 0, s, none, "F11"),
        boost::make_shared<Key>(c, Qt::Key_F12, 0, s, none, "F12"),
        //boost::make_shared<Key>(c, Qt::Key_Eject, 0, esc_box, none, "") TODO: newer Qt
        boost::make_shared<Key>(c, 0x010000b9, 0, esc_box, none, "")
    };
    for(unsigned i = 0; i < sizeof(top_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = top_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        } 
    }
    
    const key_ptr_t num_row[] = {
        boost::make_shared<Key>(c, Qt::Key_section, 0, b, none, "",  shift, "±"),
        boost::make_shared<Key>(c, Qt::Key_1, 0,       b, none, "1", shift, "!"),
        boost::make_shared<Key>(c, Qt::Key_2, 0,       b, none, "2", shift, "@"),
        boost::make_shared<Key>(c, Qt::Key_3, 0,       b, none, "3", shift, ""),
        boost::make_shared<Key>(c, Qt::Key_4, 0,       b, none, "4", shift, "$"),
        boost::make_shared<Key>(c, Qt::Key_5, 0,       b, none, "5", shift, "%"),
        boost::make_shared<Key>(c, Qt::Key_6, 0,       b, none, "6", shift, "^"),
        // oops... make_shared only accepts up to 10 arguments
        key_ptr_t(new Key(c, Qt::Key_7, 0,       b, none, "7", shift, "&", num, "7")),
        key_ptr_t(new Key(c, Qt::Key_8, 0,       b, none, "8", shift, "*", num, "8")),
        key_ptr_t(new Key(c, Qt::Key_9, 0,       b, none, "9", shift, "(", num, "9")),
        key_ptr_t(new Key(c, Qt::Key_0, 0,       b, none, "0", shift, ")", num, "/")),
        key_ptr_t(new Key(c, Qt::Key_Minus, 0,   b, none, "-", shift, "_", num, "=")),
        boost::make_shared<Key>(c, Qt::Key_Equal, 0,   b, none, "=", shift, "+"),
        boost::make_shared<Key>(c, Qt::Key_Backspace, 0, bksp_box, none, "")
    };
    pos.x = 0;
    pos.y -= Key_H/2 + Key_P;
    for(unsigned i = 0; i < sizeof(num_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = num_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        } 
    }

    const key_ptr_t qw_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Tab, 0, bksp_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_Q, 0,   b, none, "q", shift, "Q"),
        boost::make_shared<Key>(c, Qt::Key_W, 0,   b, none, "w", shift, "W"),
        boost::make_shared<Key>(c, Qt::Key_E, 0,   b, none, "e", shift, "E"),
        boost::make_shared<Key>(c, Qt::Key_R, 0,   b, none, "r", shift, "R"),
        boost::make_shared<Key>(c, Qt::Key_T, 0,   b, none, "t", shift, "T"),
        boost::make_shared<Key>(c, Qt::Key_Y, 0,   b, none, "y", shift, "Y"),
        boost::make_shared<Key>(c, Qt::Key_U, 0,   b, none, "u", shift, "U"),
        boost::make_shared<Key>(c, Qt::Key_I, 0,   b, none, "i", shift, "I"),
        boost::make_shared<Key>(c, Qt::Key_O, 0,   b, none, "o", shift, "O"),
        boost::make_shared<Key>(c, Qt::Key_P, 0,   b, none, "p", shift, "P"),
        boost::make_shared<Key>(c, Qt::Key_BraceLeft, 0,  b, none, "[", shift, "{"),
        boost::make_shared<Key>(c, Qt::Key_BraceRight, 0, b, none, "]", shift, "}"),
        boost::make_shared<ReturnKey>(c, Qt::Key_Return, 0,  b, enter_step, enter_descend)
    };
    pos.x = 0;
    pos.y -= Key_H + Key_P;
    for(unsigned i = 0; i < sizeof(qw_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = qw_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        } 
    }

    const key_ptr_t as_row[] = {
        boost::make_shared<Key>(c, Qt::Key_CapsLock, 0, cps_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_A, 0, b, none, "a", shift, "A"),
        boost::make_shared<Key>(c, Qt::Key_S, 0, b, none, "s", shift, "S"),
        boost::make_shared<Key>(c, Qt::Key_D, 0, b, none, "d", shift, "D"),
        boost::make_shared<Key>(c, Qt::Key_F, 0, b, none, "f", shift, "F"),
        boost::make_shared<Key>(c, Qt::Key_G, 0, b, none, "g", shift, "G"),
        boost::make_shared<Key>(c, Qt::Key_H, 0, b, none, "h", shift, "H"),
        boost::make_shared<Key>(c, Qt::Key_J, 0, b, none, "j", shift, "J"),
        boost::make_shared<Key>(c, Qt::Key_K, 0, b, none, "k", shift, "K"),
        boost::make_shared<Key>(c, Qt::Key_L, 0, b, none, "l", shift, "L"),
        boost::make_shared<Key>(c, Qt::Key_Semicolon, 0,  b, none, ";", shift, ":"),
        boost::make_shared<Key>(c, Qt::Key_Apostrophe, 0, b, none, "'", shift, "\""),
        boost::make_shared<Key>(c, Qt::Key_Backslash, 0,  b, none, "\\", shift, "|")
    };
    pos.x = 0;
    pos.y -= Key_H + Key_P;
    for(unsigned i = 0; i < sizeof(as_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = as_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        }
    }

    const key_ptr_t zx_row[] = {
        boost::make_shared<Key>(c, Qt::Key_Shift, 0,   lshift_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_AsciiTilde, 0, b, none, "`", shift, "~"), //? 
        boost::make_shared<Key>(c, Qt::Key_Z, 0,       b, none, "z", shift, "Z"),
        boost::make_shared<Key>(c, Qt::Key_X, 0,       b, none, "x", shift, "X"),
        boost::make_shared<Key>(c, Qt::Key_C, 0,       b, none, "c", shift, "C"),
        boost::make_shared<Key>(c, Qt::Key_V, 0,       b, none, "v", shift, "V"),
        boost::make_shared<Key>(c, Qt::Key_B, 0,       b, none, "b", shift, "B"),
        boost::make_shared<Key>(c, Qt::Key_N, 0,       b, none, "n", shift, "N"),
        boost::make_shared<Key>(c, Qt::Key_M, 0,       b, none, "m", shift, "M"),
        boost::make_shared<Key>(c, Qt::Key_Comma, Qt::Key_Less,     b, none, ",", shift, "<"),
        boost::make_shared<Key>(c, Qt::Key_Period, Qt::Key_Greater, b, none, ".", shift, ">"),
        boost::make_shared<Key>(c, Qt::Key_Slash, Qt::Key_Question, b, none, "/", shift, "?"),
        boost::make_shared<Key>(c, Qt::Key_Shift, 0,   rshift_box, none, "")
    };
    pos.x = 0;
    pos.y -= Key_H + Key_P;
    for(unsigned i = 0; i < sizeof(zx_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = zx_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        }
    }
    
    const key_ptr_t fn_row[] = {
        boost::make_shared<Key>(c, Qt::Key_NumLock, 0, fn_box, none, ""), // !!!
        boost::make_shared<Key>(c, Qt::Key_Meta, 0,    b, none, "ctrl"),
        boost::make_shared<Key>(c, Qt::Key_Alt, 0,     b, none, "opt"),
        boost::make_shared<Key>(c, Qt::Key_Control, 0, lshift_box, none, "cmd"),
        boost::make_shared<Key>(c, Qt::Key_Space, 0,   space_box, none, ""),
        boost::make_shared<Key>(c, Qt::Key_Control, 0, lshift_box, none, "cmd"),
        boost::make_shared<Key>(c, Qt::Key_Enter, 0,   b, none, "")
    };
    pos.x = 0;
    pos.y -= Key_H + Key_P;
    for(unsigned i = 0; i < sizeof(fn_row) / sizeof(key_ptr_t); i++){
        key_ptr_t k = fn_row[i];
        k->m_pos = pos;
        pos.x += k->bbox().w() + Key_P;
        foreach(keycode_t kc, k->keyCodes()){
            if(r.count(kc)) warning() << "duplicate keycode" << kc;
            r.insert(std::make_pair(kc, k));
        } 
    }

    key_ptr_t lk = boost::make_shared<Key>(c, Qt::Key_Left, 0,    s, none, "");
    key_ptr_t uk = boost::make_shared<Key>(c, Qt::Key_Up, 0,      s, none, "");
    key_ptr_t dk = boost::make_shared<Key>(c, Qt::Key_Down, 0,    s, none, "");
    key_ptr_t rk = boost::make_shared<Key>(c, Qt::Key_Right, 0,   s, none, "");

    pos.y -= Key_H/2;
    lk->m_pos = pos;
    pos.x += Key_W + Key_P;
    dk->m_pos = pos;
    uk->m_pos = pos + Point(0, Key_H/2 + Key_P/2);
    pos.x += Key_W + Key_P;
    rk->m_pos = pos;
    
    foreach(keycode_t kc, lk->keyCodes()) r.insert(std::make_pair(kc, lk));
    foreach(keycode_t kc, uk->keyCodes()) r.insert(std::make_pair(kc, uk));
    foreach(keycode_t kc, dk->keyCodes()) r.insert(std::make_pair(kc, dk));
    foreach(keycode_t kc, rk->keyCodes()) r.insert(std::make_pair(kc, rk));

    return r;
}


OverKey::OverKey(container_ptr_t parent)
    : Renderable(parent), Container(), m_bbox(), m_layout(), m_actions(),
      m_last_kp_time(std::numeric_limits<float>::min()),
      m_prev_kp_time(std::numeric_limits<float>::min()), 
      m_held_keys(),
      m_last_no_keys_time(0.0f),
      m_delayed_callbacks(),
      m_last_cb_processing_time(0.0f){
    m_layout = appleEnGBKeys(this);
    m_bbox = _calcBbox();
}

BBox OverKey::bbox(){
    return m_bbox;
}

Point OverKey::referUp(Point const& p) const{
    return m_context->referUp(p + m_pos);
}

void OverKey::postRedraw(float delay){
    m_context->postRedraw(delay);
}

void OverKey::postMenu(menu_ptr_t m, Point const& tlp, bool p){
    m_context->postMenu(m, tlp, p);
}

void OverKey::postText(const std::string &text, const std::string &font)
{
	m_context->postText(text,font);
}

void OverKey::removeMenu(menu_ptr_t m){
    m_context->removeMenu(m);
}

void OverKey::remove(renderable_ptr_t){
    error() << __func__ << __LINE__ << "unimplemented";
}

bool OverKey::keyPressEvent(KeyEvent const& event){
    KeyBind b(event.key(), event.modifiers());
    m_current_modifiers = b.modifiers;
    layout_map_t::iterator i;

    // remove any key-release callbacks for this key - don't want to update the
    // graphic to show the key as being released now that it has been pressed
    // again
    cancelDelayedCallbacks(mkStr() << "keyrel" << b.keycode);
    
    // NB: these iterators are re-used below
    std::pair<layout_map_t::iterator, layout_map_t::iterator> eq = m_layout.equal_range(b.keycode);
    for(i = eq.first; i != eq.second; i++)
        i->second->state(keystate_e::pressed);
    
    if(/*event.text().size() && */!event.isAutoRepeat()){
        m_prev_kp_time = m_last_kp_time;
        m_last_kp_time = _fnow();
    }/*else if(b.modifiers && !m_held_keys.size()){
        // if there is only a modifier being held
        m_last_kp_time = _fnow();
    }*/
    
    // modifier keys are counted as 'held' based on m_current_modifiers
    // TODO: don't do this, and simulate key events based on changes to
    // m_current_modifiers instead
    /* errr... scrap that: TODO still stands though
    switch(b.keycode){
        case Qt::Key_Shift:
        case Qt::Key_Control:
        case Qt::Key_Meta:
        case Qt::Key_Alt:
        case Qt::Key_AltGr:
            break;
        default:*/
            for(i = eq.first; i != eq.second; i++){
                m_held_keys.insert(i->second);
                debug(2) << BashColour::Red << "key pressed" << i->second;
            }
    /*}*/
    
    if(!event.isAutoRepeat())
        debug() << "keyPressEvent:" << event.text().toStdString() << "k=" << b.keycode
                << "ms=" << b.modifiers << "lt=" << m_last_kp_time << "pt=" << m_prev_kp_time;

    if(m_actions.count(b)){
        m_actions[b]->onPress();
        return true;
    }
    return false;
}

bool OverKey::keyReleaseEvent(KeyEvent const& event){
    KeyBind b(event.key(), event.modifiers());

    m_current_modifiers = b.modifiers;
    layout_map_t::iterator i;
    std::pair<layout_map_t::iterator, layout_map_t::iterator> eq = m_layout.equal_range(b.keycode);
    
    bool typing = m_last_kp_time - m_prev_kp_time < Typing_Max_Delta;
    if(typing)
        for(i = eq.first; i != eq.second; i++)
            i->second->state(keystate_e::released);
    else{
        for(i = eq.first; i != eq.second; i++)
            postDelayedCallback(
                mkStr() << "keyrel" << b.keycode,
                boost::bind(&Key::state, i->second, keystate_e::released),
                Release_Delay
            );
        // FIXME... since for now delayed callbacks are processed in draw():
        if(eq.first != eq.second)
            postRedraw(Release_Delay);
    }
    
    if(m_layout.count(b.keycode)){
        for(i = eq.first; i != eq.second; i++){
            m_held_keys.erase(i->second);
            debug(2) << BashColour::Brown << "key released" << i->second;
        }
        if(!m_held_keys.size()){
            m_last_no_keys_time = _fnow();
            debug(2) << BashColour::Green << "all keys released";
        }
    }
        
    debug() << "keyReleaseEvent:" << event.text().toStdString() << b.keycode
            << b.modifiers << m_last_kp_time << m_prev_kp_time
            << m_held_keys.size();


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
    KeyBind binding(key, mods);
    registerKey(binding, act);
}

void OverKey::draw(drawtype_e::e flags){
    if(flags & drawtype_e::picking)
        return;

    processDelayedCallbacks();
    const float fac = _alphaFrac();
    
    if(fac > 0.0f){
        debug(9) << BashColour::White << "alpha frac:" << fac;

        glColor(OK_BG_Colour * Colour(1, fac));
        glBox(bbox(), BG_Border);
        
        glTranslatef(0, 0, 0.1);
        
        // don't draw keys with more than one code twice
        std::set<key_ptr_t> drawn;
        typedef std::pair<keycode_t, key_ptr_t> layout_value_t;
        foreach(layout_value_t r, m_layout){
            const key_ptr_t k = r.second;
            if(drawn.count(k))
                continue;
            drawn.insert(k);
            glPushMatrix();
            glTranslatef(k->m_pos);
            glPushMatrix();
            k->draw(m_current_modifiers, Colour(1, fac));
            glPopMatrix();
            glCheckError();
            foreach(keycode_t kc, k->keyCodes()){
                KeyBind potential_kb(kc, m_current_modifiers);
                if(m_actions.count(potential_kb)){
                    BBox draw_area = k->bbox();
                    draw_area *= 0.85f;
                    draw_area.max.y -= draw_area.h() * 0.45;
                    glColor(Key_Decal_Colours[k->state()]);
                    m_actions[potential_kb]->drawDecal(draw_area);
                }
            }
            glPopMatrix();
        }
        
        if(_fnow() - m_last_kp_time < 3.0f)
            postRedraw(1.0/20);
    }
}

void OverKey::postDelayedCallback(std::string const& name,
                                  callback_t const& foo,
                                  float delay_secs){
    m_delayed_callbacks.insert(_Callback(_fnow() + delay_secs, name, foo));
}

void OverKey::cancelDelayedCallbacks(std::string const& name){
    cb_map_by_name_t& callbacks_by_name = m_delayed_callbacks.get<name_index>();
    cb_map_by_name_t::iterator b = callbacks_by_name.lower_bound(name);
    cb_map_by_name_t::iterator e = callbacks_by_name.upper_bound(name);
    callbacks_by_name.erase(b, e);
}

void OverKey::processDelayedCallbacks(){
    cb_map_by_time_t::iterator b, e, i;
    cb_map_by_time_t& callbacks_by_time = m_delayed_callbacks.get<time_index>();
    float now = _fnow();
    int count = 0;

    b = callbacks_by_time.lower_bound(m_last_cb_processing_time);
    e = callbacks_by_time.upper_bound(now);
    for(i = b; i != e; i++, count++)
        i->callback();
    
    callbacks_by_time.erase(b, e);
    m_last_cb_processing_time = now;

    if(count){
        debug() << now << "processed" << count << "delayed callbacks";
        debug() << m_delayed_callbacks.size() << "remain";
    }
}

BBox OverKey::_calcBbox() const{
    BBox r;
    layout_map_t::const_iterator i;
    for(i = m_layout.begin(); i != m_layout.end(); i++)
        r |= i->second->bbox() + i->second->m_pos;
    r.min -= BG_Border;
    r.max += BG_Border;
    return r;
}

namespace bpt = boost::posix_time;
float OverKey::_fnow() const{
    static bpt::ptime fnow_base;
    float now = 0.0f;
    if(fnow_base == bpt::not_a_date_time)
        fnow_base = bpt::microsec_clock::local_time();
    bpt::ptime current_time = bpt::microsec_clock::local_time();
    bpt::time_duration diff = current_time - fnow_base;
    now = diff.total_seconds();
    now += float(diff.fractional_seconds()) / bpt::time_duration::ticks_per_second();
    return now;
}

float OverKey::_alphaFrac() const{
    // based on the current time, how opaque should things be drawn?
    float now = _fnow();
    float last_time = 0.0f;
    float delta = 0.0f;
    float nokey_delta = 0.0f;
    float r = 0.0f;
    bool typing = false;
    const bool keys_actually_held = m_held_keys.size();
    bool key_held = false;
        
    // sanitise previous time... just makes things easier:
    last_time = 0;
    if(boost::math::isnan(m_last_kp_time))
        last_time = 0;
    else
        last_time = m_last_kp_time;
    delta = now - last_time;
    nokey_delta = now - m_last_no_keys_time;

    // shift modifier counts as typing
    if(last_time - m_prev_kp_time < Typing_Max_Delta &&
       !(m_current_modifiers & ~Qt::ShiftModifier))
        typing = true;
    
    debug(3) << "typing=" << typing << "delta=" << delta
             << "lt=" << last_time << "pt=" << m_prev_kp_time;
    r = 0.0f;

    if(keys_actually_held && nokey_delta > 0.8){
        key_held = true;
        // slow things down
        delta /= 2;
    }else if(!keys_actually_held){
        // speed things up
        delta *= 2;
    }
        
    if(!typing){
        /*
         *   alpha mul
         *      ^
         *   1 -|     oooo
         *      |    o    ooo
         * .75 -|   o        oo
         *      |   o          oo
         * .5  -|   o            ooo
         *      |  o                oooo
         * .25 -|  o                    oooo
         *      | o                         oooo
         *   0 -oo------------------------------oooooooooooooo-> time since 
         *      '    '    '    '    '    '    '    '    '    '    last key
         *     0.0  0.4  0.8  1.2  1.6  2.0  2.4  2.8
         */
        if     (delta < 0.16) r = 0.02;
        else if(delta < 0.24) r = 0.02 + 0.23 * (delta-0.16) / 0.08;
        else if(delta < 0.32) r = 0.25 + 0.50 * (delta-0.24) / 0.08;
        else if(delta < 0.40) r = 0.75 + 0.25 * (delta-0.32) / 0.08;
        else if(delta < 0.48) r = 1.00;
        else if(delta < 0.80) r = 1.00 - 0.25 * (delta-0.48) / 0.32; 
        else if(delta < 2.00) r = 0.75 - 0.50 * (delta-0.80) / 1.20;
        else if(delta < 2.68) r = 0.25 - 0.25 * (delta-2.00) / 0.68;
        else /*delta > 2.68*/ r = 0.00;
    }
    
    //
    if(/*key_held*/ keys_actually_held && (delta > 0.4 || keys_actually_held > 1)){
        r = std::max(0.8f, r);
    }
    
    return r;
}


