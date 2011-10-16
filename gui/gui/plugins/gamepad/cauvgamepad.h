/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <QObject>
#include <QTimer>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

namespace cauv {

    class AUV;
    class GamepadInput;
    class XBoxInput;
    class PlaystationInput;

    class CauvGamepad : public QObject
    {
        Q_OBJECT
    public:
        CauvGamepad(boost::shared_ptr<XBoxInput> input, boost::shared_ptr<AUV> auv);
        CauvGamepad(boost::shared_ptr<PlaystationInput> input, boost::shared_ptr<AUV> auv);
        ~CauvGamepad() {}

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

        void toggleAutopilotControl(bool pressed);

        void stop(bool);

    protected Q_SLOTS:
        void update();

    protected:
        void startTimer();

        boost::shared_ptr<AUV> m_auv;
        boost::shared_ptr<GamepadInput> m_gamepadInput;
        boost::scoped_ptr<QTimer> m_timer;
        
        float m_bearingRate;
        float m_pitchRate;
        float m_forwardSpeed;
        float m_strafeSpeed;
        float m_depthRate;
        bool m_dirty;
        bool m_autopilotControl;

    };

} // namespace cauv

#endif // GAMEPAD_H
