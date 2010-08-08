#ifndef __OVERKEY_RENDERABLE_H__
#define __OVERKEY_RENDERABLE_H__

#include <map>
#include <string>

#include <Qt>

#include <boost/function.hpp>

#include "../renderable.h"
#include "../container.h"

namespace pw{
namespace ok{

typedef int keycode_t;
typedef Qt::KeyboardModifiers modifiers_t;

class Action{
    public:
        typedef boost::function<void()> f_t;
        
        static f_t null_f;

        Action(f_t const& on_press_f = null_f,
                 f_t const& on_release_f = null_f,
                 std::string const& descr = "",
                 renderable_ptr_t decal = renderable_ptr_t());

        void onPress() const;
        void onRelease() const;

    private:
        const f_t on_press_f;
        const f_t on_release_f;
        std::string m_descr;
        renderable_ptr_t m_decal;
};

class Key: public Renderable{
    public:
        typedef std::map<Qt::KeyboardModifiers, renderable_ptr_t> textmap_t;
        Key(container_ptr_t, keycode_t const&, BBox const&, textmap_t const& text);
        Key(container_ptr_t, keycode_t const&, BBox const&,
            Qt::KeyboardModifiers m1 = 0, std::string const& t1 = "",
            Qt::KeyboardModifiers m2 = 0, std::string const& t2 = "",
            Qt::KeyboardModifiers m3 = 0, std::string const& t3 = "",
            Qt::KeyboardModifiers m4 = 0, std::string const& t4 = "");

        virtual ~Key();

        virtual void draw(drawtype_e::e);
        virtual void draw(Qt::KeyboardModifiers const& mods);

        virtual BBox bbox();

        keycode_t const& keyCode() const;
        void state(keystate_e::e s);

    protected:
        keystate_e::e m_state;

    private:
        void centerText();
        
        keycode_t m_keycode;
        textmap_t m_text;
        BBox m_box;
};

struct KeyBind{
    bool operator<(KeyBind const& r) const;
    keycode_t keycode;
    modifiers_t modifiers;
};


class OverKey: public Renderable,
               public Container{
    public:
        typedef std::multimap<keycode_t, key_ptr_t> layout_map_t;
        typedef std::map<KeyBind, action_ptr_t> action_map_t;

        OverKey(container_ptr_t parent);

        // Implement container:
        virtual Point referUp(Point const& p) const;
        virtual void postRedraw();
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position,
                              bool pressed=false);
        virtual void removeMenu(menu_ptr_t);
        virtual void remove(renderable_ptr_t); 

        // Handle key presses
        virtual bool keyPressEvent(QKeyEvent *event);
        virtual bool keyReleaseEvent(QKeyEvent *event);
        
        // Interface for registering keys
        void registerKey(KeyBind const&, action_ptr_t);
        void registerKey(keycode_t const&, modifiers_t const&, action_ptr_t);

        void draw(drawtype_e::e);
        
        void setLayout(layout_map_t const&);

    private:
        layout_map_t m_layout;
        action_map_t m_actions;
        Qt::KeyboardModifiers m_current_modifiers;
};

} // namespace ok
} // namespace pw

#endif // ndef __OVERKEY_RENDERABLE_H__

