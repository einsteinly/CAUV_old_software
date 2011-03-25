#ifdef GAMEPAD_SUPPORT

#include "xboxinput.h"

#include <iostream>

using namespace OIS;
using namespace cauv;

XBoxInput::XBoxInput(const unsigned int id) : GamepadInput(id)
{
}

bool XBoxInput::buttonPressed( const JoyStickEvent&, int button ) {
    emitButton((XBox::Buttons) button, true);
    return true;
}

bool XBoxInput::buttonReleased( const JoyStickEvent&, int button ) {
    emitButton((XBox::Buttons) button, false);
    return true;
}

bool XBoxInput::axisMoved( const JoyStickEvent &arg, int axis )
{
    float value = 0;
    if((arg.state.mAxes[axis].abs > 6000 || arg.state.mAxes[axis].abs < -6000 )) {
        value = ((float)arg.state.mAxes[axis].abs) / 32768.f; // 32768 is the maximum short value
    }

    switch (axis) {
        case XBox::Joy_L_X:
            Q_EMIT Joy_L_X(value);
            break;
        case XBox::Joy_L_Y:
            Q_EMIT Joy_L_Y(value);
            break;
        case XBox::Joy_R_X:
            Q_EMIT Joy_R_X(value);
            break;
        case XBox::Joy_R_Y:
            Q_EMIT Joy_R_Y(value);
            break;
        default:
            throw "Axis does not exist";
    }

    return true;
}

bool XBoxInput::povMoved( const JoyStickEvent &arg, int pov )
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

bool XBoxInput::vector3Moved( const JoyStickEvent &, int )
{
        //not used for XBox
        return true;
}


void XBoxInput::printIt(bool it) const {
    std::cout << "Button value: " << it << std::endl;
}

void XBoxInput::printIt(int it) const {
    std::cout << "Axis value: " << it << std::endl;
}

void XBoxInput::emitButton( XBox::Buttons button, bool state ) {

    std::cout << "x box button: " << button << std::endl;

    switch(button){
    case XBox::A:
        Q_EMIT A(state);
        break;
    case XBox::B:
        Q_EMIT B(state);
        break;
    case XBox::X:
        Q_EMIT X(state);
        break;
    case XBox::Y:
        Q_EMIT Y(state);
        break;
    case XBox::Back:
        Q_EMIT Back(state);
        break;
    case XBox::Start:
        Q_EMIT Start(state);
        break;
    case XBox::LT:
        Q_EMIT LT(state);
        break;
    case XBox::LB:
        Q_EMIT LB(state);
        break;
    case XBox::RT:
        Q_EMIT RT(state);
        break;
    case XBox::RB:
        Q_EMIT RB(state);
        break;
    case XBox::JoyLClick:
        Q_EMIT JoyLClick(state);
        break;
    case XBox::JoyRClick:
        Q_EMIT JoyRClick(state);
        break;
    default:
        throw "Unknown button";
    }
}


#endif //GAMEPAD_SUPPORT
