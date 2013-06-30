/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GAMEPADINPUT_H
#define GAMEPADINPUT_H

#include <QObject>

#include <OIS/OISPrereqs.h>
#include <OIS/OISJoyStick.h>

namespace OIS {
    class JoyStick;
    class JoyStickEvent;
    class InputManager;
}

namespace cauv{
    namespace gui {

        class GamepadInput : public QObject, public OIS::JoyStickListener
        {
            Q_OBJECT

            public Q_SLOTS:
            void processEvents();

        public:
            explicit GamepadInput(const std::string& vendor = "");

            static OIS::DeviceList listDevices();
            static int getNumDevices();

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

    } // namespace gui
} // namespace cauv

#endif // GAMEPADINPUT_H
