#ifndef PLAYSTATIONINPUT_H
#define PLAYSTATIONINPUT_H

#include "gamepadinput.h"

namespace Playstation {
    enum Buttons {
        X = 2, Triangle = 0, Square = 3, Circle = 1,
        R1 = 7, R2 = 5, L1 = 6, L2 = 4,
        JoyLClick = 10, JoyRClick = 11,
        Select = 8, Start = 9
    };
    enum POV {
        Left = 4096, Right = 256, Down = 16, Up = 1
    };
    enum Axes {
        Joy_L_X = 0,
        Joy_L_Y = 1,
        Joy_R_X = 4,
        Joy_R_Y = 3,
    };
}


class PlaystationInput : public GamepadInput
{
    Q_OBJECT

Q_SIGNALS:
    void X(bool pressed);
    void Triangle(bool pressed);
    void Square(bool pressed);
    void Circle(bool pressed);
    void R1(bool pressed);
    void R2(bool pressed);
    void L1(bool pressed);
    void L2(bool pressed);
    void JoyLClick(bool pressed);
    void JoyRClick(bool pressed);
    void Select(bool pressed);
    void Start(bool pressed);
    void Left();
    void Right();
    void Up();
    void Down();
    void Centered();
    void Joy_L_X(int value);
    void Joy_L_Y(int value);
    void Joy_R_X(int value);
    void Joy_R_Y(int value);


public Q_SLOTS:
    void printIt(bool);
    void printIt(int);

public:
    PlaystationInput(const unsigned int id);

    bool buttonPressed( const OIS::JoyStickEvent &arg, int button );
    bool buttonReleased( const OIS::JoyStickEvent &arg, int button );
    bool axisMoved( const OIS::JoyStickEvent &arg, int axis );
    bool povMoved( const OIS::JoyStickEvent &arg, int pov );
    bool vector3Moved( const OIS::JoyStickEvent &arg, int index);

protected:
    void emitButton( Playstation::Buttons button, bool state );
};

#endif // PLAYSTATIONINPUT_H
