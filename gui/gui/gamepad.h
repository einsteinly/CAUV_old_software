#define GAMEPAD_SUPPRT 1
#ifdef GAMEPAD_SUPPORT

#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <gamepad/playstationinput.h>

#include <boost/shared_ptr.hpp>

namespace cauv {

    class AUV;

    class CauvGamepad : public PlaystationInput
    {
        Q_OBJECT
    public:
        CauvGamepad(const unsigned int id, boost::shared_ptr<AUV> auv);

    public Q_SLOTS:
        void forward(bool go);
        void backward(bool go);
        void strafeLeft(bool go);
        void strafeRight(bool go);
        void up(bool go);
        void down(bool go);

        void forward(float speed);
        void backward(float speed);
        void strafeLeft(float rate);
        void strafeRight(float rate);
        void pitchUp(float rate);
        void pitchDown(float rate);
        void yawLeft(float rate);
        void yawRight(float rate);

        void stop(bool);

    protected:
        boost::shared_ptr<AUV> m_auv;

        float m_bearingRate;
        float m_pitchRate;

    protected Q_SLOTS:
        void updateByRates();

    };

} // namespace cauv

#endif // GAMEPAD_H

#endif //GAMEPAD_SUPPORT
