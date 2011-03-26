#ifdef GAMEPAD_SUPPORT

#ifndef XBOXINPUT_H
#define XBOXINPUT_H

#include "gamepadinput.h"

namespace cauv{

namespace XBox {
    enum Buttons {
        X = 7, Y = 8, B = 6, A = 5,
        RB = 10, RT = 2, LB = 9, LT = 1,
        JoyLClick = 13, JoyRClick = 14,
        Start = 11, Back = 4
    };
    enum POV {
        Left = 2, Right = 3, Down = 1, Up = 0
    };
    enum Axes {
        Joy_L_X = 0,
        Joy_L_Y = 1,
        Joy_R_X = 3,
        Joy_R_Y = 4,
    };
}


class XBoxInput : public GamepadInput
{
    Q_OBJECT

Q_SIGNALS:
    void X(bool pressed);
    void Y(bool pressed);
    void A(bool pressed);
    void B(bool pressed);
    void RT(bool pressed);
    void RB(bool pressed);
    void LT(bool pressed);
    void LB(bool pressed);
    void JoyLClick(bool pressed);
    void JoyRClick(bool pressed);
    void Back(bool pressed);
    void Start(bool pressed);
    void Left();
    void Right();
    void Up();
    void Down();
    void Centered();
    void Joy_L_X(float value);
    void Joy_L_Y(float value);
    void Joy_R_X(float value);
    void Joy_R_Y(float value);


public Q_SLOTS:
    void printIt(bool) const;
    void printIt(int) const;

public:
    explicit XBoxInput(const unsigned int id);

    bool buttonPressed( const OIS::JoyStickEvent &arg, int button ) ;
    bool buttonReleased( const OIS::JoyStickEvent &arg, int button ) ;
    bool axisMoved( const OIS::JoyStickEvent &arg, int axis ) ;
    bool povMoved( const OIS::JoyStickEvent &arg, int pov ) ;
    bool vector3Moved( const OIS::JoyStickEvent &arg, int index) ;

protected:
    void emitButton( XBox::Buttons button, bool state );
};

} // namespace cauv

#endif // XBOXINPUT_H

#endif //GAMEPAD_SUPPORT
