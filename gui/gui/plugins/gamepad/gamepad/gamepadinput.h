/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
            explicit GamepadInput(const std::string vendor = "");

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
