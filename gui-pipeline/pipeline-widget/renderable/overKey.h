#ifndef __OVERKEY_RENDERABLE_H__
#define __OVERKEY_RENDERABLE_H__

#include <Qt>

#include <boost/function.hpp>

#include "../renderable.h"

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
        Key(container_ptr_t c, double const& w, double const& h, textmap_t const& text=textmap_t());
        virtual ~Key();

        virtual void draw(drawtype_e::e);
        virtual void draw(Qt::KeyboardModifiers const& mods);

        virtual BBox bbox();

    private:
        textmap_t m_text;
        BBox m_box;
};

struct KeyBind{
    bool operator<(KeyBind const& r) const;
    keycode_t keycode;
    modifiers_t modifiers;
};


class OverKey: public Renderable{
    public:
        typedef std::map<keycode_t, key_ptr_t> layout_map_t;
        typedef std::map<KeyBind, action_ptr_t> action_map_t;

        OverKey(container_ptr_t container);

        virtual bool keyPressEvent(QKeyEvent *event);
        virtual bool keyReleaseEvent(QKeyEvent *event);
        
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

