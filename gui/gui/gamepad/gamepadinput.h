#ifndef GAMEPADINPUT_H
#define GAMEPADINPUT_H


#include <qobject.h>

#include <X11/Xlib.h>

#include <OIS/OISInputManager.h>
#include <OIS/OISException.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISEvents.h>

namespace cauv{

class GamepadInput : public QObject, public OIS::JoyStickListener
{
    Q_OBJECT

public Q_SLOTS:
    void processEvents();

public:
    GamepadInput(const unsigned int id);

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

    virtual void handleNonBuffered();
};

} // namespace cauv

#endif // GAMEPADINPUT_H
