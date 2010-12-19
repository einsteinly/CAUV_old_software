#include "playstationinput.h"

#include <iostream>

using namespace OIS;
using namespace cauv;

PlaystationInput::PlaystationInput(const unsigned int id) : GamepadInput(id)
{
}

bool PlaystationInput::buttonPressed( const JoyStickEvent&, int button ) {
    emitButton((Playstation::Buttons) button, true);
    return true;
}

bool PlaystationInput::buttonReleased( const JoyStickEvent&, int button ) {
    emitButton((Playstation::Buttons) button, false);
    return true;
}

bool PlaystationInput::axisMoved( const JoyStickEvent &arg, int axis )
{
    int value = 0;
    if((arg.state.mAxes[axis].abs > 4000 || arg.state.mAxes[axis].abs < -4000 )) {
        value = arg.state.mAxes[axis].abs;
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
            throw "Axis does not exist";
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

bool PlaystationInput::vector3Moved( const JoyStickEvent &, int )
{
        //not used for playstation
        return true;
}


void PlaystationInput::printIt(bool it){
    std::cout << "Button value: " << it << std::endl;
}

void PlaystationInput::printIt(int it){
    std::cout << "Axis value: " << it << std::endl;
}

void PlaystationInput::emitButton( Playstation::Buttons button, bool state ) {
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
    case Playstation::JoyLClick:
        Q_EMIT JoyLClick(state);
        break;
    case Playstation::JoyRClick:
        Q_EMIT JoyRClick(state);
        break;
    default:
        throw "Unknown button";
    }
}
