#ifdef GAMEPAD_SUPPORT

#ifndef GAMEPADINPUT_H
#define GAMEPADINPUT_H

#include <QObject>
#include <OIS/OISJoyStick.h>

namespace OIS {
    class JoyStick;
    class JoyStickEvent;
    class InputManager;
}

namespace cauv{

    class GamepadInput : public QObject, public OIS::JoyStickListener
    {
        Q_OBJECT

        public Q_SLOTS:
        void processEvents();

    public:
        explicit GamepadInput(const unsigned int id);

        static std::string listDevices();

        virtual bool buttonPressed( const OIS::JoyStickEvent &arg, int button );
        virtual bool buttonReleased( const OIS::JoyStickEvent &arg, int button );
        virtual bool axisMoved( const OIS::JoyStickEvent &arg, int axis );
        virtual bool povMoved( const OIS::JoyStickEvent &arg, int pov );
        virtual bool vector3Moved( const OIS::JoyStickEvent &arg, int index);

    protected:
        static OIS::InputManager *m_input_manager;
        OIS::JoyStick *m_controller;

        static OIS::InputManager* getInputSystem();

        virtual void handleNonBuffered() const;
    };

} // namespace cauv

#endif // GAMEPADINPUT_H

#endif //GAMEPAD_SUPPORT
