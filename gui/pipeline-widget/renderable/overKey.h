/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __OVERKEY_RENDERABLE_H__
#define __OVERKEY_RENDERABLE_H__

#include <map>
#include <string>
#include <vector>
#include <set>

#include <Qt>

#include <boost/function.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>

#include "../renderable.h"
#include "../container.h"

namespace cauv{
namespace gui{
namespace pw{
namespace ok{

typedef int keycode_t;
typedef Qt::KeyboardModifiers modifiers_t;

namespace bmi = boost::multi_index;

class Action{
    public:
        typedef boost::function<void()> f_t;
        
        static f_t null_f;

        Action(f_t const& on_press_f = null_f,
                 f_t const& on_release_f = null_f,
                 const std::string& descr = "",
                 renderable_ptr_t decal = renderable_ptr_t());

        void onPress() const;
        void onRelease() const;

        void drawDecal(BBox const& area);

    private:
        const f_t on_press_f;
        const f_t on_release_f;
        std::string m_descr;
        renderable_ptr_t m_decal;
};

class Key: public Renderable{
    public:
        typedef std::map<Qt::KeyboardModifiers, renderable_ptr_t> textmap_t;
        Key(container_ptr_t, keycode_t const&, keycode_t const&, BBox const&, textmap_t const& text);
        Key(container_ptr_t, keycode_t const&, keycode_t const&, BBox const&,
            Qt::KeyboardModifiers m1 = 0, const std::string& t1 = "",
            Qt::KeyboardModifiers m2 = 0, const std::string& t2 = "",
            Qt::KeyboardModifiers m3 = 0, const std::string& t3 = "",
            Qt::KeyboardModifiers m4 = 0, const std::string& t4 = "");

        virtual ~Key();

        virtual void draw(drawtype_e::e);
        virtual void draw(Qt::KeyboardModifiers const& mods, Colour const& mul);

        virtual BBox bbox();

        std::vector<keycode_t> const& keyCodes() const;
        void state(keystate_e::e s);
        keystate_e::e state() const;

    protected:
        keystate_e::e m_state;

    private:
        void centerText();
        
        std::vector<keycode_t> m_keycodes;
        textmap_t m_text;
        BBox m_box;
};

struct KeyBind{
    KeyBind(keycode_t const& kc, modifiers_t const& m);
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

        BBox bbox();

        // Implement container:
        virtual Point referUp(Point const& p) const;
        virtual void postRedraw(float delay);
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position,
                              bool pressed=false);
		virtual void postText(const std::string &text, const std::string &font);
        virtual void removeMenu(menu_ptr_t);
        virtual void remove(renderable_ptr_t); 

        // Handle key presses
        virtual bool keyPressEvent(KeyEvent const& event);
        virtual bool keyReleaseEvent(KeyEvent  const& event);
        
        // Interface for registering keys
        void registerKey(KeyBind const&, action_ptr_t);
        void registerKey(keycode_t const&, modifiers_t const&, action_ptr_t);

        void draw(drawtype_e::e);
        
        void setLayout(layout_map_t const&);
        

        /**** callback mechanism that should be somewhere else.... ****/

        // TODO: FIXME FIRST NOW SOON OR SOMETIME: this callback mechanism, and
        // the way that it is used relies on callbacks being added, processed
        // and removed from one thread: there is no locking. Further, the
        // postRedraw() function must be used to make sure that draw() is
        // called as/just after any events need to be processed - and that
        // function will ONLY work from a thread with a Qt event loop (because
        // of the way that it is implemented in PipelineWidget), so. caveat
        // viator: you have been warned.

        // TODO: add this interface to Container; implement using QTimer in PipelineWidget 

        /**** types ****/
        typedef boost::function<void()> callback_t;
        struct _Callback{
            _Callback(float const& t, const std::string& n, callback_t const& cb)
                : time(t), name(n), callback(cb){
            }

            float time;
            std::string name;
            callback_t callback;
        };
        struct time_index{};
        struct name_index{};
        typedef bmi::multi_index_container<
            _Callback,
            bmi::indexed_by<
                bmi::ordered_non_unique<
                    bmi::tag<time_index>, bmi::member<_Callback, float, &_Callback::time>
                >,
                bmi::ordered_non_unique<
                    bmi::tag<name_index>, bmi::member<_Callback, std::string, &_Callback::name>
                >
            >
        > cb_map_t;
        typedef cb_map_t::index<name_index>::type cb_map_by_name_t;
        typedef cb_map_t::index<time_index>::type cb_map_by_time_t;
        /**** end callback mechanism types ****/


        /**** callback mechanism that should be somewhere else.... ****/
        virtual void postDelayedCallback(const std::string& name,
                                         callback_t const& foo,
                                         float delay_secs);
        virtual void processDelayedCallbacks();
        virtual void cancelDelayedCallbacks(const std::string& name);
        /**** end callback mechanism functions ****/

    private:
        BBox _calcBbox() const;
        float _fnow() const;
        float _alphaFrac() const;
        
        BBox m_bbox;
        layout_map_t m_layout;
        action_map_t m_actions;
        Qt::KeyboardModifiers m_current_modifiers;
        
        float m_last_kp_time;
        float m_prev_kp_time;
        std::set<key_ptr_t> m_held_keys;
        float m_last_no_keys_time;
        
        /**** callback mechanism that should be somewhere else.... ****/
        cb_map_t m_delayed_callbacks;
        float m_last_cb_processing_time;
        /**** end callback mechanism data ****/
};

} // namespace ok
} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __OVERKEY_RENDERABLE_H__

