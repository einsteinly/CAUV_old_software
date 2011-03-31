#ifdef GAMEPAD_SUPPORT

#include "xboxinput.h"

#include <iostream>
#include <stdexcept>

using namespace OIS;
using namespace cauv;

XBoxInput::XBoxInput(const std::string vendor) : GamepadInput(vendor)
{
}

bool XBoxInput::buttonPressed( const JoyStickEvent& event, int button ) {
    if(emitButton((XBox::Buttons) button, true))
        return true;
    else return GamepadInput::buttonPressed(event, button);
}

bool XBoxInput::buttonReleased( const JoyStickEvent& event, int button ) {
    if(emitButton((XBox::Buttons) button, false))
        return true;
    else return GamepadInput::buttonReleased(event, button);
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
    case XBox::Trigger_L:
        Q_EMIT Trigger_L(value);
        break;
    case XBox::Trigger_R:
        Q_EMIT Trigger_R(value);
        break;
    default:
        GamepadInput::axisMoved(arg, axis);
    }

    return true;
}

bool XBoxInput::emitButton( XBox::Buttons button, bool state ) {

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
    case XBox::XBox:
        Q_EMIT XBox(state);
        break;
    case XBox::Back:
        Q_EMIT Back(state);
        break;
    case XBox::Start:
        Q_EMIT Start(state);
        break;
    case XBox::LB:
        Q_EMIT LB(state);
        break;
    case XBox::RB:
        Q_EMIT RB(state);
        break;
    case XBox::Up:
        Q_EMIT Up(state);
        break;
    case XBox::Down:
        Q_EMIT Down(state);
        break;
    case XBox::Left:
        Q_EMIT Left(state);
        break;
    case XBox::Right:
        Q_EMIT Right(state);
        break;
    case XBox::Joy_L_Click:
        Q_EMIT Joy_L_Click(state);
        break;
    case XBox::Joy_R_Click:
        Q_EMIT Joy_R_Click(state);
        break;
    default:
        return false;
    }
    return true;
}


#endif //GAMEPAD_SUPPORT
