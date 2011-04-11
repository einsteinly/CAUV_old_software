#include "playstationinput.h"

#include <iostream>

using namespace OIS;
using namespace cauv;

PlaystationInput::PlaystationInput(const std::string vendor) : GamepadInput(vendor)
{
}

bool PlaystationInput::buttonPressed( const JoyStickEvent& event, int button ) {
    if(emitButton((Playstation::Buttons) button, true))
        return true;
    else return GamepadInput::buttonPressed(event, button);
}

bool PlaystationInput::buttonReleased( const JoyStickEvent& event, int button ) {
    if(emitButton((Playstation::Buttons) button, false))
        return true;
    else return GamepadInput::buttonReleased(event, button);
}

bool PlaystationInput::axisMoved( const JoyStickEvent &arg, int axis )
{
    float value = 0;
    if((arg.state.mAxes[axis].abs > 6000 || arg.state.mAxes[axis].abs < -6000 )) {
        value = ((float)arg.state.mAxes[axis].abs) / 32768.f; // 32768 is the maximum short value
    }

    switch (axis) {
    case Playstation::Joy_L_X:
        Q_EMIT Joy_L_X(value);
        break;
    case Playstation::Joy_L_Y:
        Q_EMIT Joy_L_Y(value);
        break;
    case Playstation::Joy_R_X:
        Q_EMIT Joy_R_X(value);
        break;
    case Playstation::Joy_R_Y:
        Q_EMIT Joy_R_Y(value);
        break;
    default:
        return GamepadInput::axisMoved(arg, axis);
    }

    return true;
}

bool PlaystationInput::povMoved( const JoyStickEvent &arg, int pov )
{
    if( arg.state.mPOV[pov].direction & Pov::North ) //Going up
        Q_EMIT Up();
    else if( arg.state.mPOV[pov].direction & Pov::South ) //Going down
        Q_EMIT Down();

    if( arg.state.mPOV[pov].direction & Pov::East ) //Going right
        Q_EMIT Right();
    else if( arg.state.mPOV[pov].direction & Pov::West ) //Going left
        Q_EMIT Left();

    if( arg.state.mPOV[pov].direction == Pov::Centered ) //stopped/centered out
        Q_EMIT Centered();

    return true;
}


bool PlaystationInput::emitButton( Playstation::Buttons button, bool state ) {
    switch(button){
    case Playstation::Circle:
        Q_EMIT Circle(state);
        break;
    case Playstation::Triangle:
        Q_EMIT Triangle(state);
        break;
    case Playstation::Square:
        Q_EMIT Square(state);
        break;
    case Playstation::X:
        Q_EMIT X(state);
        break;
    case Playstation::Select:
        Q_EMIT Select(state);
        break;
    case Playstation::Start:
        Q_EMIT Start(state);
        break;
    case Playstation::L1:
        Q_EMIT L1(state);
        break;
    case Playstation::L2:
        Q_EMIT L2(state);
        break;
    case Playstation::R1:
        Q_EMIT R1(state);
        break;
    case Playstation::R2:
        Q_EMIT R2(state);
        break;
    case Playstation::Joy_L_Click:
        Q_EMIT Joy_L_Click(state);
        break;
    case Playstation::Joy_R_Click:
        Q_EMIT Joy_R_Click(state);
        break;
    default:
        return false;
    }
    return true;
}
