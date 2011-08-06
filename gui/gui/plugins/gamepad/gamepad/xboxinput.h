#ifndef XBOXINPUT_H
#define XBOXINPUT_H

#include "gamepadinput.h"

namespace cauv{
    
    namespace XBox {
        enum Buttons {
            X = 7, Y = 8, B = 6, A = 5,
            LB = 9, RB = 10,
            Back = 4, Start = 11, XBox = 12,
            Joy_L_Click = 13, Joy_R_Click = 14,
            Left = 2, Right = 3, Up = 0, Down = 1,
                                     };
        enum Axes {
            Joy_L_X = 0,
            Joy_L_Y = 1,
            Joy_R_X = 3,
            Joy_R_Y = 4,
            Trigger_L = 2,
            Trigger_R = 5,
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
        void XBox(bool pressed);
        void RB(bool pressed);
        void LB(bool pressed);
        void Joy_L_Click(bool pressed);
        void Joy_R_Click(bool pressed);
        void Back(bool pressed);
        void Start(bool pressed);
        void Left(bool pressed);
        void Right(bool pressed);
        void Up(bool pressed);
        void Down(bool pressed);
        void Joy_L_X(float value);
        void Joy_L_Y(float value);
        void Joy_R_X(float value);
        void Joy_R_Y(float value);
        void Trigger_L(float value);
        void Trigger_R(float value);
        
    public:
        explicit XBoxInput(const std::string vendor);
        
        bool buttonPressed( const OIS::JoyStickEvent &arg, int button ) ;
        bool buttonReleased( const OIS::JoyStickEvent &arg, int button ) ;
        bool axisMoved( const OIS::JoyStickEvent &arg, int axis ) ;
        
    protected:
        bool emitButton( XBox::Buttons button, bool state );
    };
    
} // namespace cauv

#endif // XBOXINPUT_H
