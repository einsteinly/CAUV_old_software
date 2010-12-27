#ifndef AUV_MODEL_H_INCLUDED
#define AUV_MODEL_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <generated/messages_fwd.h>
#include <generated/messages.h>

#include <common/data_stream.h>

namespace cauv{

struct autopilot_params_t {
    float kP, kI, kD, scale;

    autopilot_params_t(float kP, float kI, float kD, float scale) : kP(kP), kI(kI), kD(kD), scale(scale) {
    }

    autopilot_params_t() {
    }

    friend std::ostream& operator <<(std::ostream &os,const autopilot_params_t &obj);
};

struct depth_calibration_t {
    float foreMultiplier, afteMultiplier, foreOffset, aftOffset;

    depth_calibration_t(float foreMultiplier, float afteMultiplier, float foreOffset, float aftOffset) :
    foreMultiplier(foreMultiplier), afteMultiplier(afteMultiplier),
    foreOffset(foreOffset), aftOffset(aftOffset) {
    }

    depth_calibration_t() {
    }

    friend std::ostream& operator <<(std::ostream &os,const depth_calibration_t &obj);
};

struct sonar_params_t {
    int direction, width, gain, range, radialRes, angularRes;

    sonar_params_t(int direction, int width, int gain, int radialRes, int angularRes) :
    direction(direction), width(width), gain(gain), radialRes(radialRes), angularRes(angularRes) {
    }

    sonar_params_t() {
    }

    friend std::ostream& operator <<(std::ostream &os,const sonar_params_t &obj);
};

std::ostream& operator <<(std::ostream &os,const int8_t &value);


class AUV {
public:

    /**
     * Represents a motor in the AUV. All motors available are listed in the
     * Motors class Setting the motor speed (via setSpeed) will fire the
     * speedChanged signal to update interested objects. Calling the updateSpeed
     * method will disable the controller before setting the speed so that the
     * new motor speed is not sent to the AUV
     *
     * @author Andy Pritchard
     */

    class Motor : public MutableDataStream<int8_t> {
    public:

        Motor(const MotorID::e &id, const std::string name) : MutableDataStream<int8_t>(name), m_id(id) {
        };

        virtual const MotorID::e &getID();

    protected:
        const MotorID::e &m_id;
    };

    struct Motors {
        boost::shared_ptr< Motor > prop;
        boost::shared_ptr< Motor > hbow;
        boost::shared_ptr< Motor > vbow;
        boost::shared_ptr< Motor > hstern;
        boost::shared_ptr< Motor > vstern;

        Motors() :
        prop(boost::make_shared<Motor > (MotorID::Prop, "Prop")),
        hbow(boost::make_shared<Motor > (MotorID::HBow, "H Bow")),
        vbow(boost::make_shared<Motor > (MotorID::VBow, "V Bow")),
        hstern(boost::make_shared<Motor > (MotorID::HStern, "H Stern")),
        vstern(boost::make_shared<Motor > (MotorID::VStern, "V Stern")) {
        }
    } motors;

    /**
     * Represents an autopilot in the AUV. The type of target (e.g. a heading or
     * depth) that the autopilot aims for is passed in using generics. This is
     * then sent on the AUV when the state is changed to enabled, or if it is
     * already enabled when the target is set.
     *
     * @author Andy Pritchard
     *
     * @param <T>
     *            The type of the target the autopilot uses
     */

    template <class T>

    class Autopilot : public MutableDataStream<T> {
    public:

        Autopilot(const std::string name, const T initialTarget) :
        MutableDataStream<T>(name),
        params(boost::make_shared< MutableDataStream<autopilot_params_t> >("Params", this)),
        kP(boost::make_shared< DataStream<float> >("kP", this)),
        kI(boost::make_shared< DataStream<float> >("kI", this)),
        kD(boost::make_shared< DataStream<float> >("kD", this)),
        scale(boost::make_shared< DataStream<float> >("scale", this)),
        enabled(boost::make_shared< MutableDataStream<bool> >("Enabled", this)) {
            this->set(initialTarget);
        };

        boost::shared_ptr< MutableDataStream<autopilot_params_t> > params;
        boost::shared_ptr< DataStream<float> > kP;
        boost::shared_ptr< DataStream<float> > kI;
        boost::shared_ptr< DataStream<float> > kD;
        boost::shared_ptr< DataStream<float> > scale;
        boost::shared_ptr< MutableDataStream<bool> > enabled;
    };

    struct Autopilots {
        boost::shared_ptr< Autopilot<float> > bearing;
        boost::shared_ptr< Autopilot<float> > depth;
        boost::shared_ptr< Autopilot<float> > pitch;

        Autopilots() :
        bearing(boost::make_shared< Autopilot<float> >("Bearing", 0)),
        depth(boost::make_shared< Autopilot<float> >("Depth", 0)),
        pitch(boost::make_shared< Autopilot<float> >("Pitch", 0)) {
        }
    } autopilots;

    /**
     * Log streams are availible here
     * there is a special level stream combined with these that allows
     * the debug level to written / read
     *
     * @author Andy Pritchard
     */
    struct Logs {
        boost::shared_ptr< DataStream<std::string> > debug;
        boost::shared_ptr< DataStream<std::string> > trace;
        boost::shared_ptr< DataStream<std::string> > error;
        boost::shared_ptr< MutableDataStream<int32_t> > level;

        Logs() :
        debug(boost::make_shared< DataStream<std::string> >("Debug")),
        trace(boost::make_shared< DataStream<std::string> >("Trace")),
        error(boost::make_shared< DataStream<std::string> >("Error")),
        level(boost::make_shared< MutableDataStream<int32_t> >("Level")) {
        }
    } logs;

    /**
     * Camera streams
     *
     * @author Andy Pritchard
     */
    class Camera : public DataStream<Image> {
    public:

        Camera(const std::string name) : DataStream<Image>(name) {
        }
    };

    class Sonar : public Camera {
    public:

        Sonar(const std::string name) : Camera(name), params(boost::make_shared<MutableDataStream< sonar_params_t > >("Params", this)) {
        }

        boost::shared_ptr< MutableDataStream< sonar_params_t > > params;
    };

    struct Cameras {
        boost::shared_ptr< Camera > forward;
        boost::shared_ptr< Camera > down;
        boost::shared_ptr< Sonar > sonar;

        Cameras() :
        forward(boost::make_shared< Camera > ("Forward Camera")),
        down(boost::make_shared< Camera > ("Downward Camera")),
        sonar(boost::make_shared< Sonar > ("Sonar")) {
        }
    } cameras;

    struct Sensors {
        boost::shared_ptr< DataStream<uint16_t> > pressure_fore;
        boost::shared_ptr< DataStream<uint16_t> > pressure_aft;
        boost::shared_ptr<MutableDataStream<depth_calibration_t> > depth_calibration;
        boost::shared_ptr<DataStream<float> > depth;
        boost::shared_ptr<DataStream<float> > yaw;
        boost::shared_ptr<DataStream<float> > pitch;
        boost::shared_ptr<DataStream<float> > roll;
        boost::shared_ptr<DataStream<floatYPR> > orientation;

        Sensors() : pressure_fore(boost::make_shared< DataStream<uint16_t> >("Pressure Fore")),
        pressure_aft(boost::make_shared< DataStream<uint16_t> >("Pressure Aft")),
        depth_calibration(boost::make_shared<MutableDataStream<depth_calibration_t> >("Depth Calibration")),
        depth(boost::make_shared<DataStream<float> >("Depth")),
        yaw(boost::make_shared<DataStream<float> >("Yaw")),
        pitch(boost::make_shared<DataStream<float> >("Pitch")),
        roll(boost::make_shared<DataStream<float> >("Roll")),
        orientation(boost::make_shared<DataStream<floatYPR> >("Orientation")) {
        }
    } sensors;

    virtual ~AUV() {
    }

};

} // namespace cauv

#endif // AUV_MODEL_H_INCLUDED
