/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "gamepadinput.h"

#include <sstream>
#include <iostream>
#include <stdexcept>

#include <OIS/OISInputManager.h>
#include <OIS/OISEvents.h>

#include <X11/Xlib.h>

using namespace OIS;
using namespace cauv;
using namespace cauv::gui;

InputManager * GamepadInput::m_input_manager = 0;

GamepadInput::GamepadInput(const std::string vendor)
{
    m_input_manager = GamepadInput::getInputSystem();

    m_controller = (JoyStick*)m_input_manager->createInputObject( OISJoyStick, true, vendor);
    m_controller->setEventCallback( this );
}

InputManager * GamepadInput::getInputSystem() {
    if(m_input_manager) return m_input_manager;

    ParamList pl;
    Display *xDisp = 0;

    //Connects to default X window
    if( !(xDisp = XOpenDisplay(0)) )
        throw std::runtime_error("Error opening X!");

    //Create a window
    // make it an InputOnly window as we don't want anything visible to the user
    Window xWin = XCreateWindow(xDisp,DefaultRootWindow(xDisp), 0,0, 1,1, 0, CopyFromParent, InputOnly, CopyFromParent, 0, 0);

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
     InputManager * mng  = InputManager::createInputSystem(pl);

     //Lets enable all addons that were compiled in:
     mng->enableAddOnFactory(InputManager::AddOn_All);

     return mng;
}

bool GamepadInput::buttonPressed( const JoyStickEvent &arg, int button ) {
    std::cout << std::endl << arg.device->vendor() << ". Button Pressed # " << button << std::endl;
    return true;
}
bool GamepadInput::buttonReleased( const JoyStickEvent &arg, int button ) {
    std::cout << std::endl << arg.device->vendor() << ". Button Released # " << button << std::endl;
    return true;
}
bool GamepadInput::axisMoved( const JoyStickEvent &arg, int axis ) 
{
    //Provide a little dead zone
    if( arg.state.mAxes[axis].abs > 4000 || arg.state.mAxes[axis].abs < -4000 )
        std::cout << std::endl << arg.device->vendor() << ". Axis # " << axis << " Value: " << arg.state.mAxes[axis].abs << std::endl;
    return true;
}
bool GamepadInput::povMoved( const JoyStickEvent &arg, int pov ) 
{
    std::cout << std::endl << arg.device->vendor() << ". POV" << pov << " ";

    if( arg.state.mPOV[pov].direction & Pov::North ) //Going up
        std::cout <<  arg.state.mPOV[pov].direction << "North" << std::endl;

    else if( arg.state.mPOV[pov].direction & Pov::South ) //Going down
        std::cout << arg.state.mPOV[pov].direction << "South" << std::endl;

    if( arg.state.mPOV[pov].direction & Pov::East ) //Going right
        std::cout <<  arg.state.mPOV[pov].direction << "East" << std::endl;

    else if( arg.state.mPOV[pov].direction & Pov::West ) //Going left
        std::cout <<  arg.state.mPOV[pov].direction << "West" << std::endl;

    if( arg.state.mPOV[pov].direction == Pov::Centered ) //stopped/centered out
        std::cout <<  arg.state.mPOV[pov].direction << "Centered" << std::endl;
    return true;
}

bool GamepadInput::vector3Moved( const JoyStickEvent &arg, int index) 
{
    std::cout.precision(2);
    std::cout.flags(std::ios::fixed | std::ios::right);
    std::cout << std::endl << arg.device->vendor() << ". Orientation # " << index
            << " X Value: " << arg.state.mVectors[index].x
            << " Y Value: " << arg.state.mVectors[index].y
            << " Z Value: " << arg.state.mVectors[index].z << std::endl;
    std::cout.precision();
    std::cout.flags();
    return true;
}

void GamepadInput::processEvents(){
    try {
        m_controller->capture();
        if( !m_controller->buffered() )
            handleNonBuffered();
    } catch (...){
        std::cerr << "exception caught while capuring gamepad events"<< std::endl;
    }
}

void GamepadInput::handleNonBuffered() const {
    const JoyStickState &joy = m_controller->getJoyStickState();
    for( unsigned int i = 0; i < joy.mAxes.size(); ++i ) {
        std::cout << "\nAxis " << i << " X: " << joy.mAxes[i].abs;
    }
}

DeviceList GamepadInput::listDevices() {
    InputManager *im = GamepadInput::getInputSystem();
    return im->listFreeDevices();
}

int GamepadInput::getNumDevices(){
    return GamepadInput::getInputSystem()->getNumberOfDevices( OISJoyStick );
}
