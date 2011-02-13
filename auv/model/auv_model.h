#ifndef AUV_MODEL_H_INCLUDED
#define AUV_MODEL_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>

#include <generated/messages_fwd.h>
#include <generated/messages.h>

#include <common/data_stream.h>
#include <common/data_stream_tools.h>

namespace cauv{


struct depth_calibration_t {
    float foreMultiplier, afteMultiplier, foreOffset, aftOffset;

    depth_calibration_t(float foreMultiplier, float afteMultiplier, float foreOffset, float aftOffset) :
    foreMultiplier(foreMultiplier), afteMultiplier(afteMultiplier),
    foreOffset(foreOffset), aftOffset(aftOffset) {
    }

    depth_calibration_t() : foreMultiplier(0), afteMultiplier(0),
            foreOffset(0), aftOffset(0) {
    }
};

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, depth_calibration_t const& s)
{
    os << "fore (" << s.foreOffset;
    os << ", x" << s.foreMultiplier;
    os << ")";
    os << " aft (" << s.aftOffset;
    os << ", x" << s.afteMultiplier;
    os << ")";
    return os;
}



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
        const std::string m_name;
    };

    typedef boost::unordered_map<MotorID::e, boost::shared_ptr<Motor> > motor_map;

    motor_map motors;


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

        Autopilot(const std::string name, boost::shared_ptr<DataStream<T> > actualValue, const T initialTarget, const T min, const T max, const std::string units="") :
        MutableDataStream<T>(name, units),
        kP(boost::make_shared< MutableDataStream<float> >("kP", this)),
        kI(boost::make_shared< MutableDataStream<float> >("kI", this)),
        kD(boost::make_shared< MutableDataStream<float> >("kD", this)),
        scale(boost::make_shared< MutableDataStream<float> >("scale", this)),
        enabled(boost::make_shared< MutableDataStream<bool> >("Enabled", this)),
        actual(actualValue),
        m_max(max),
        m_min(min) {
            this->update(initialTarget);
            this->enabled->update(false);
        };

        boost::shared_ptr< MutableDataStream<float> > kP;
        boost::shared_ptr< MutableDataStream<float> > kI;
        boost::shared_ptr< MutableDataStream<float> > kD;
        boost::shared_ptr< MutableDataStream<float> > scale;
        boost::shared_ptr< MutableDataStream<bool> > enabled;
        boost::shared_ptr< DataStream<T> > actual;


        T getMax(){
            return m_max;
        }

        T getMin(){
            return m_min;
        }

        void set(const T &data){
            if(data > getMax())
                MutableDataStream<T>::set(getMax());
            else if(data < getMin())
                MutableDataStream<T>::set(getMin());
            else MutableDataStream<T>::set(data);
        }

    protected:
        const T m_max;
        const T m_min;
    };

    typedef boost::unordered_map<std::string, boost::shared_ptr<Autopilot<float> > > autopilot_map;

    autopilot_map autopilots;


    /**
     * Log streams are availible here
     * there is a special level stream combined with these that allows
     * the debug level to written / read
     *
     * @author Andy Pritchard
     */

    typedef boost::unordered_map<DebugType::e, boost::shared_ptr<DataStream<std::string> > > logs_map;

    logs_map logs;

    boost::shared_ptr< MutableDataStream<int32_t> > debug_level;


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

        Sonar(const std::string name) : Camera(name),
            angularRes(boost::make_shared<MutableDataStream< int > >("Angular Resolution", this)),
            radialRes(boost::make_shared<MutableDataStream< int > >("Radial Resolution", this)),
            range(boost::make_shared<MutableDataStream< int > >("Range", this)),
            direction(boost::make_shared<MutableDataStream< int > >("Direction", this)),
            gain(boost::make_shared<MutableDataStream< int > >("Gain", this)),
            width(boost::make_shared<MutableDataStream< int > >("Width", this)) {
        }

        boost::shared_ptr< MutableDataStream< int > > angularRes;
        boost::shared_ptr< MutableDataStream< int > > radialRes;
        boost::shared_ptr< MutableDataStream< int > > range;
        boost::shared_ptr< MutableDataStream< int > > direction;
        boost::shared_ptr< MutableDataStream< int > > gain;
        boost::shared_ptr< MutableDataStream< int > > width;
    };

    typedef boost::unordered_map<CameraID::e, boost::shared_ptr<Camera> > camera_map;

    camera_map cameras;


    /**
     * Sensor streams - other sensors that dont fit into the above
     * categories
     *
     * @author Andy Pritchard
     */

    struct Sensors {
        boost::shared_ptr< DataStream<uint16_t> > pressure_fore;
        boost::shared_ptr< DataStream<uint16_t> > pressure_aft;
        boost::shared_ptr<MutableDataStream<depth_calibration_t> > depth_calibration;
        boost::shared_ptr<DataStream<float> > depth;
        boost::shared_ptr<DataStream<floatYPR> > orientation;
        boost::shared_ptr<DataStreamSplitter<cauv::floatYPR> > orientation_split;

        Sensors() : pressure_fore(boost::make_shared< DataStream<uint16_t> >("Pressure Fore")),
        pressure_aft(boost::make_shared< DataStream<uint16_t> >("Pressure Aft")),
        depth_calibration(boost::make_shared<MutableDataStream<depth_calibration_t> >("Depth Calibration")),
        depth(boost::make_shared<DataStream<float> >("Depth", "m")),
        orientation(boost::make_shared<DataStream<floatYPR> >("Orientation", "°")),
        orientation_split(boost::make_shared<DataStreamSplitter<cauv::floatYPR> >(orientation)) {
        }
    } sensors;

    AUV();
    virtual ~AUV() {
    }

};

} // namespace cauv

#endif // AUV_MODEL_H_INCLUDED
