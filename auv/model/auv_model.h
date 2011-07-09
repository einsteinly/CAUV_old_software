#ifndef AUV_MODEL_H_INCLUDED
#define AUV_MODEL_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>

#include <common/data_stream.h>
#include <common/data_stream_tools.h>
#include <common/image.h>

#include <generated/types/CameraID.h>
#include <generated/types/MotorID.h>
#include <generated/types/DebugType.h>
#include <generated/types/ScriptExecRequest.h>
#include <generated/types/ScriptResponse.h>
#include <generated/types/ProcessStatusMessage.h>
#include <generated/types/DepthCalibrationMessage.h>
#include <generated/types/LocationMessage.h>

namespace cauv{

    // some messages are used to group data in the gui but we typedef
    // them for clarity as we're not really using them as messages
    typedef ProcessStatusMessage ProcessState;
    typedef DepthCalibrationMessage DepthCalibration;
    typedef LocationMessage Location;

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

            Autopilot(const std::string name, boost::shared_ptr<DataStream<T> > actualValue, const T initialTarget, const T min, const T max, const bool wraps, const std::string units="") :
                    MutableDataStream<T>(name, units),
                    kP(boost::make_shared< MutableDataStream<float> >("kP", this)),
                    kI(boost::make_shared< MutableDataStream<float> >("kI", this)),
                    kD(boost::make_shared< MutableDataStream<float> >("kD", this)),
                    scale(boost::make_shared< MutableDataStream<float> >("scale", this)),
                    aP(boost::make_shared< MutableDataStream<float> >("aP", this)),
                    aI(boost::make_shared< MutableDataStream<float> >("aI", this)),
                    aD(boost::make_shared< MutableDataStream<float> >("aD", this)),
                    thr(boost::make_shared< MutableDataStream<float> >("thr", this)),
                    maxError(boost::make_shared< MutableDataStream<float> >("maxError", this)),
                    enabled(boost::make_shared< MutableDataStream<bool> >("Enabled", this)),
                    actual(actualValue),
                    mv(boost::make_shared< DataStream<float> >("mv", this)),
                    error(boost::make_shared< DataStream<float> >("error", this)),
                    derror(boost::make_shared< DataStream<float> >("derror", this)),
                    ierror(boost::make_shared< DataStream<float> >("ierror", this)),
                    demand(boost::make_shared<DataStreamSplitter<cauv::MotorDemand> >(boost::make_shared< DataStream<MotorDemand> >("demand", this))),
                    m_max(max),
                    m_min(min),
                    m_wraps(wraps){
                this->update(initialTarget);
                this->enabled->update(false);
            };

            boost::shared_ptr< MutableDataStream<float> > kP;
            boost::shared_ptr< MutableDataStream<float> > kI;
            boost::shared_ptr< MutableDataStream<float> > kD;
            boost::shared_ptr< MutableDataStream<float> > scale;
            boost::shared_ptr< MutableDataStream<float> > aP;
            boost::shared_ptr< MutableDataStream<float> > aI;
            boost::shared_ptr< MutableDataStream<float> > aD;
            boost::shared_ptr< MutableDataStream<float> > thr;
            boost::shared_ptr< MutableDataStream<float> > maxError;
            boost::shared_ptr< MutableDataStream<bool> > enabled;
            boost::shared_ptr< DataStream<T> > actual;

            boost::shared_ptr< DataStream<float> > mv;
            boost::shared_ptr< DataStream<float> > error;
            boost::shared_ptr< DataStream<float> > derror;
            boost::shared_ptr< DataStream<float> > ierror;
            boost::shared_ptr< DataStreamSplitter<cauv::MotorDemand> > demand;


            T getMax(){
                return m_max;
            }

            T getMin(){
                return m_min;
            }

            T wrap(T value){
                T range = getMax() - getMin();

                if(value < getMin()) {
                    return wrap(value + range);
                }
                else if(value > getMax())
                    return wrap(value - range);
                else return value;
            }

            void set(const T &data){
                if(data > getMax()) {
                    if(m_wraps) MutableDataStream<T>::set(wrap(data));
                    else MutableDataStream<T>::set(getMax());
                } else if(data < getMin()) {
                    if(m_wraps) MutableDataStream<T>::set(wrap(data));
                    else MutableDataStream<T>::set(getMin());
                }
                else MutableDataStream<T>::set(data);
            }

        protected:
            const T m_max;
            const T m_min;
            const bool m_wraps;
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
            boost::shared_ptr<MutableDataStream<DepthCalibration> > depth_calibration;
            boost::shared_ptr<DataStream<float> > depth;
            boost::shared_ptr<DataStreamSplitter<cauv::floatYPR> > orientation;
            boost::shared_ptr<DataStream<float> > esitmate_current;
            boost::shared_ptr<DataStream<float> > estimate_total;
            boost::shared_ptr<DataStream<float> > fraction_remaining;
            boost::shared_ptr<DataStream<floatXYZ> > speed;
            boost::shared_ptr<DataStream<Location> > location;

            Sensors() : pressure_fore(boost::make_shared< DataStream<uint16_t> >("Pressure Fore")),
            pressure_aft(boost::make_shared< DataStream<uint16_t> >("Pressure Aft")),
            depth_calibration(boost::make_shared<MutableDataStream<DepthCalibration> >("Depth Calibration")),
            depth(boost::make_shared<DataStream<float> >("Depth", "m")),
            orientation(boost::make_shared<DataStreamSplitter<cauv::floatYPR> >(boost::make_shared<DataStream<floatYPR> >("Orientation", "°"))),
            esitmate_current(boost::make_shared<DataStream<float> >("Current Estimiation", "Wh")),
            estimate_total(boost::make_shared<DataStream<float> >("Total Battery Consumed", "Wh")),
            fraction_remaining(boost::make_shared<DataStream<float> >("Battery Remaining", "%")),
            speed(boost::make_shared<DataStream<floatXYZ> >("Speed", "m/s")),
            location(boost::make_shared<DataStream<Location> >("Location", "lng/lat/alt"))
            {

            }
        } sensors;

        /**
     * Computer state streams - monitoring of the software and hardware state
     * categories
     *
     * @author Andy Pritchard
     */

        struct ComputerState {
            
            typedef boost::unordered_map<std::string, boost::shared_ptr<DataStream<ProcessState> > > process_map;
            
            boost::shared_ptr<DataStream<boost::shared_ptr< DataStream<ProcessState> > > > new_process_stream;

            process_map processes;

            ComputerState() : new_process_stream(boost::make_shared<DataStream<boost::shared_ptr< DataStream<ProcessState> > > >("New Processes", ""))
            {
            }
        } computer_state;



        /**
        * Debug Graphing - monitoring of the software and hardware state
        * categories
        *
        * @author Andy Pritchard
        */

           struct Debug {

               typedef boost::unordered_map<std::string, boost::shared_ptr<DataStream<float> > > graph_map;

               boost::shared_ptr<DataStream<boost::shared_ptr< DataStream<float> > > > new_graph_stream;

               graph_map graphs;

               Debug() : new_graph_stream(boost::make_shared<DataStream<boost::shared_ptr< DataStream<float> > > >("New Graph Stream", ""))
               {
               }
           } debug;

        
        /**
     * Scripting
     *
     * @author Andy Pritchard
     */

        struct Scripts {
            boost::shared_ptr<MutableDataStream<ScriptExecRequest> > scriptExec;
            boost::shared_ptr<DataStream<ScriptResponse> > scriptResponse;

            Scripts() : scriptExec(boost::make_shared<MutableDataStream<ScriptExecRequest> >("Script Execution")),
            scriptResponse(boost::make_shared<DataStream<ScriptResponse> >("Script Response")) {
            }

        } scripts;

        AUV();
        virtual ~AUV() {
        }

    };

} // namespace cauv

#endif // AUV_MODEL_H_INCLUDED
