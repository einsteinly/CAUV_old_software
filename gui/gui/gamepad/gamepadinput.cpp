#ifdef GAMEPAD_SUPPORT

#include "gamepadinput.h"

#include <sstream>
#include <iostream>

#include <OIS/OISInputManager.h>
#include <OIS/OISException.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <OIS/OISEvents.h>

#include <X11/Xlib.h>

using namespace OIS;
using namespace cauv;

InputManager * GamepadInput::m_input_manager = 0;

GamepadInput::GamepadInput(const unsigned int id)
{
    m_input_manager = GamepadInput::getInputSystem();

    //Lets enable all addons that were compiled in:
    m_input_manager->enableAddOnFactory(InputManager::AddOn_All);

    JoyStick* joys[m_input_manager->getNumberOfDevices(OISJoyStick)];

    if((int)id > m_input_manager->getNumberOfDevices(OISJoyStick)-1)
        throw "Gamepad doesn't exist!";

    for( int i = 0; i < m_input_manager->getNumberOfDevices(OISJoyStick); ++i )
    {
        joys[i] = (JoyStick*)m_input_manager->createInputObject( OISJoyStick, true );
        joys[i]->setEventCallback( this );
        std::cout << "\n\nCreating Joystick " << (i + 1)
            << "\n\tAxes: " << joys[i]->getNumberOfComponents(OIS_Axis)
            << "\n\tSliders: " << joys[i]->getNumberOfComponents(OIS_Slider)
            << "\n\tPOV/HATs: " << joys[i]->getNumberOfComponents(OIS_POV)
            << "\n\tButtons: " << joys[i]->getNumberOfComponents(OIS_Button)
            << "\n\tVector3: " << joys[i]->getNumberOfComponents(OIS_Vector3);
   }

    m_controller = joys[id];
}

InputManager * GamepadInput::getInputSystem() {
    if(m_input_manager) return m_input_manager;

    ParamList pl;
    Display *xDisp = 0;

    //Connects to default X window
    if( !(xDisp = XOpenDisplay(0)) )
        OIS_EXCEPT(E_General, "Error opening X!");

    //Create a window
    // make it an InputOnly window as we don't want anything visible to the user
    Window xWin = XCreateWindow(xDisp,DefaultRootWindow(xDisp), 0,0, 100,100, 0, CopyFromParent, InputOnly, CopyFromParent, 0, 0);

    //bind our connection to that window
    XMapWindow(xDisp, xWin);

    //Select what events we want to listen to locally
    XSelectInput(xDisp, xWin, StructureNotifyMask);
    XEvent event;
    do
    {
        XNextEvent(xDisp, &event);
    } while(event.type != MapNotify);

    std::ostringstream wnd;
    wnd << xWin;

    pl.insert(std::make_pair(std::string("WINDOW"), wnd.str()));

    //This never returns null.. it will raise an exception on errors
    return InputManager::createInputSystem(pl);
}

bool GamepadInput::buttonPressed( const JoyStickEvent &arg, int button ) {
        std::cout << std::endl << arg.device->vendor() << ". Button Pressed # " << button;
        return true;
}
bool GamepadInput::buttonReleased( const JoyStickEvent &arg, int button ) {
        std::cout << std::endl << arg.device->vendor() << ". Button Released # " << button;
        return true;
}
bool GamepadInput::axisMoved( const JoyStickEvent &arg, int axis ) 
{
        //Provide a little dead zone
        if( arg.state.mAxes[axis].abs > 4000 || arg.state.mAxes[axis].abs < -4000 )
                std::cout << std::endl << arg.device->vendor() << ". Axis # " << axis << " Value: " << arg.state.mAxes[axis].abs;
        return true;
}
bool GamepadInput::povMoved( const JoyStickEvent &arg, int pov ) 
{
        std::cout << std::endl << arg.device->vendor() << ". POV" << pov << " ";

        if( arg.state.mPOV[pov].direction & Pov::North ) //Going up
                std::cout <<  arg.state.mPOV[pov].direction << "North";

        else if( arg.state.mPOV[pov].direction & Pov::South ) //Going down
                std::cout << arg.state.mPOV[pov].direction << "South";

        if( arg.state.mPOV[pov].direction & Pov::East ) //Going right
                std::cout <<  arg.state.mPOV[pov].direction << "East";

        else if( arg.state.mPOV[pov].direction & Pov::West ) //Going left
                std::cout <<  arg.state.mPOV[pov].direction << "West";

        if( arg.state.mPOV[pov].direction == Pov::Centered ) //stopped/centered out
                std::cout <<  arg.state.mPOV[pov].direction << "Centered";
        return true;
}

bool GamepadInput::vector3Moved( const JoyStickEvent &arg, int index) 
{
        std::cout.precision(2);
        std::cout.flags(std::ios::fixed | std::ios::right);
        std::cout << std::endl << arg.device->vendor() << ". Orientation # " << index
                << " X Value: " << arg.state.mVectors[index].x
                << " Y Value: " << arg.state.mVectors[index].y
                << " Z Value: " << arg.state.mVectors[index].z;
        std::cout.precision();
        std::cout.flags();
        return true;
}

void GamepadInput::processEvents(){
    m_controller->capture();
    if( !m_controller->buffered() )
        handleNonBuffered();
}

void GamepadInput::handleNonBuffered() const {
    const JoyStickState &joy = m_controller->getJoyStickState();
    for( unsigned int i = 0; i < joy.mAxes.size(); ++i ) {
        std::cout << "\nAxis " << i << " X: " << joy.mAxes[i].abs;
    }
}

std::string GamepadInput::listDevices() {
    std::stringstream str;

    InputManager *im = GamepadInput::getInputSystem();

    //List all devices
    str << "\n Available devices: \n";
    DeviceList list = im->listFreeDevices();
    for( DeviceList::iterator i = list.begin(); i != list.end(); ++i ) {
            if(i->first == OISJoyStick){
                str << "\n\tDevice: " << "Gamepad" << " Vendor: " << i->second;
            }
        }
    str << "\n\n End of device list. \n\n";
    return str.str();
}

#endif //GAMEPAD_SUPPORT
