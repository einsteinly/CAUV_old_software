package cauv.auv;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Vector;

import cauv.auv.CommunicationController.AUVConnectionObserver;
import cauv.auv.MessageSocket.ConnectionStateObserver;
import cauv.gui.ScreenView;
import cauv.types.CameraID;
import cauv.types.Image;
import cauv.types.MotorDemand;
import cauv.types.MotorID;
import cauv.types.floatYPR;

import com.trolltech.qt.QSignalEmitter;

public class AUV extends QSignalEmitter {

    static Vector<DataStream<?>> allStreams = new Vector<DataStream<?>>();
    
    public static Vector<DataStream<?>> getAllStreams() {
        return allStreams;
    }
    
    public class DataStream<T> extends QSignalEmitter {
        
        public Signal1<DataStream<T>> onChange = new Signal1<DataStream<T>>();
        
        T latest;
        
        String name;
        
        public DataStream(String name) {
            this.name = name;
            
            allStreams.add(this);
        }
        
        public String getName() {
            return name;
        }
        
        public void update(T data){
            boolean state = AUV.this.controller.getEnabled();
            AUV.this.controller.disable();
            this.set(data);
            if(state)
                AUV.this.controller.enable();
        }
        
        public void set(T data) {
            this.latest = data;
            this.onChange.emit(this);
        }
        
        public T get() {
            return latest;
        }
        
        @Override
        public String toString() {
            return name;
        }
    }
    
    public class MemorisingDataStream<T> extends DataStream<T> {
        Vector<T> history = new Vector<T>();
        
        int maximum;
        
        public MemorisingDataStream(String name, int maximum) {
            super(name);
            this.maximum = maximum;
        }
        
        public void set(T data) {
            this.history.add(data);
            
            if(!history.isEmpty() && history.size() > maximum)
                history.remove(0);
            
            super.set(data);
        };
        
        public void clear(){
            history.clear();
        }
        
        public Vector<T> getHistory() {
            return history;
        }
    }
    
    
    public abstract class MultipleDataStreamEmitter extends QSignalEmitter {
        
        public HashMap<String, DataStream<?>> dataStreams = new HashMap<String, DataStream<?>>();
        
        protected void addStream(DataStream<?> data){
            this.dataStreams.put(data.getName(), data);
        }
        
        protected DataStream<?> getStream(String name){
            return dataStreams.get(name);
        }
        
        public HashMap<String, DataStream<?>> getDataStreams() {
            return dataStreams;
        }
    }
   
    
    /**
     * Represents a motor in the AUV. All motors available are listed in the
     * Motors class Setting the motor speed (via setSpeed) will fire the
     * speedChanged signal to update interested objects. Calling the updateSpeed
     * method will disable the controller before setting the speed so that the
     * new motor speed is not sent to the AUV
     * 
     * @author Andy Pritchard
     */
    public class Motor extends DataStream<Integer> {
        
        protected MotorID id;

        public Signal1<Integer> speedChanged  = new Signal1<Integer>();
        
        public Motor(MotorID id) {
            super(id.name());
            this.id = id;
            this.latest = 0;
        }

        public MotorID getID() {
            return id;
        }
        
        @Override
        public void set(Integer data) {
            this.speedChanged.emit(data);
            super.set(data);
        }
    }

    public class Motors {
        public final Motor PROP = new Motor(MotorID.Prop);
        public final Motor HBOW = new Motor(MotorID.HBow);
        public final Motor VBOW = new Motor(MotorID.VBow);
        public final Motor HSTERN = new Motor(MotorID.HStern);
        public final Motor VSTERN = new Motor(MotorID.VStern);
    }

    public Motors motors = new Motors();

    
    public void stopAllMotors(){
    	this.motors.HBOW.set(0);
    	this.motors.HSTERN.set(0);
    	this.motors.VBOW.set(0);
    	this.motors.VSTERN.set(0);
    	this.motors.PROP.set(0);

        this.autopilots.DEPTH.setEnabled(false);
        this.autopilots.PITCH.setEnabled(false);
        this.autopilots.YAW.setEnabled(false);
    }
    
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
    public class Autopilot<T> extends MultipleDataStreamEmitter{

        
        public MemorisingDataStream<MotorDemand> demandsStream;
        public MemorisingDataStream<Float> errorStream;
        public MemorisingDataStream<Float> derrorStream;
        public MemorisingDataStream<Float> ierrorStream;
        public MemorisingDataStream<Float> mvStream;
        
        public MemorisingDataStream<T> targetStream;
        public DataStream<Boolean> enabledStream;
        public MemorisingDataStream<Float> KpStream;
        public MemorisingDataStream<Float> KiStream;
        public MemorisingDataStream<Float> KdStream;
        public MemorisingDataStream<Float> scaleStream;
        
        public Signal1<Autopilot<T>> targetChanged = new Signal1<Autopilot<T>>();
        public Signal1<Boolean> enabledChanged = new Signal1<Boolean>();
        public Signal0 paramsChanged = new Signal0();
        
        String name;
        
        public Autopilot(String name, T initialTarget) {
            
            errorStream = new MemorisingDataStream<Float>(name + " Error", 2000);
            derrorStream = new MemorisingDataStream<Float>(name + " dError", 2000);
            ierrorStream = new MemorisingDataStream<Float>(name + " iError", 2000);
            mvStream = new MemorisingDataStream<Float>(name + " mv", 2000);
            demandsStream = new MemorisingDataStream<MotorDemand>(name + " Demands", 2000);
            
            targetStream = new MemorisingDataStream<T>(name + " Target", 2000);
            enabledStream = new DataStream<Boolean>(name + " Enabled");
            
            KpStream = new MemorisingDataStream<Float>(name + " Kp", 2000);
            KiStream = new MemorisingDataStream<Float>(name + " Ki", 2000);
            KdStream = new MemorisingDataStream<Float>(name + " Kd", 2000);
            scaleStream = new MemorisingDataStream<Float>(name + " Scale", 2000);
            
            
            
            addStream(demandsStream);
            addStream(targetStream);
            addStream(enabledStream);
            addStream(KpStream);
            addStream(KiStream);
            addStream(KdStream);
            addStream(scaleStream);
            
            targetStream.latest = initialTarget;
		}
        
        public T getTarget() {
            return targetStream.latest;
        }

        public void setTarget(T target) {
            if (getEnabled()) {
                if(!this.getTarget().equals(target))
                {
                    targetStream.set(target);
                    targetChanged.emit(this);
                }
            }
        }

        public void updateTarget(T target) {
            AUV.this.controller.disable();
            targetStream.update(target);
            targetChanged.emit(this);
            AUV.this.controller.enable();
        }
        
        public boolean getEnabled() {
            return (enabledStream.latest == null)? true : enabledStream.latest;
        }

        public void setEnabled(boolean state) {
            enabledStream.set(state);
            enabledChanged.emit(state);
        }

        public void updateEnabled(boolean state) {
            AUV.this.controller.disable();
            enabledChanged.emit(state);
            enabledStream.update(state);
            AUV.this.controller.enable();
        }

        public float getKp() {
            return (KpStream.latest == null) ? 0 : KpStream.latest;
        }
        
        public float getKi() {
            return (KiStream.latest == null) ? 0 : KiStream.latest;
        }
        
        public float getKd() {
            return (KdStream.latest == null) ? 0 : KdStream.latest;
        }

        public float getScale(){
            return (scaleStream.latest == null) ? 0 : scaleStream.latest;
        }
        
        public MotorDemand getDemands() {
            return demandsStream.latest;
        }
        
        public void updateControllerState(float mv, float error, float derror, float ierror, MotorDemand demand){
            demandsStream.update(demand);
            mvStream.update(mv);
            errorStream.update(error);
            derrorStream.update(derror);
            ierrorStream.update(ierror);
        }
        
        public void setParams(float Kp, float Ki, float Kd, float scale) {
            KpStream.set(Kp);
            KiStream.set(Ki);
            KdStream.set(Kd);
            scaleStream.set(scale);
            paramsChanged.emit();
        }

        public void updateParams(float Kp, float Ki, float Kd, float scale) {
            AUV.this.controller.disable();
            KpStream.update(Kp);
            KiStream.update(Ki);
            KdStream.update(Kd);
            scaleStream.update(scale);
            paramsChanged.emit();
            AUV.this.controller.enable();
        }
    }

    public class Autopilots {
        public final Autopilot<Float> DEPTH = new Autopilot<Float>("Depth", 0.0f);
        public final Autopilot<Float> YAW = new Autopilot<Float>("Yaw", 0.0f);
        public final Autopilot<Float> PITCH = new Autopilot<Float>("Pitch", 0.0f);
    }

    public Autopilots autopilots = new Autopilots();

    /**
     * Represents a camera in the AUV.
     * 
     * @author Andy Pritchard
     * 
     */
    public class Camera extends DataStream<Image> {

        protected CameraID id;

        public Camera(CameraID id) {
            super(id.name());
            this.id = id;
        }
    }

    public class Sonar extends AUV.Camera {
        
        public Signal6<Integer,Integer,Integer,Integer,Integer,Integer> paramsChanged = 
            new Signal6<Integer,Integer,Integer,Integer,Integer,Integer>();
        
        protected int direction;
        protected int width;
        protected int gain;
        protected int range;
        protected int radialRes;
        protected int angularRes;
        
        public Sonar(CameraID id) {
            super(id);
        }
        
        public int getDirection(){
            return direction;
        }
        
        public int getAngularRes() {
            return angularRes;
        }
        
        public int getGain() {
            return gain;
        }
        
        public int getRadialRes() {
            return radialRes;
        }
        
        public int getRange() {
            return range;
        }
        
        public int getWidth() {
            return width;
        }
        
        public void setParams(int direction, int width, int gain, int range, int radialRes, int angularRes){
            this.direction = direction;
            this.width = width;
            this.gain = gain;
            this.range = range;
            this.radialRes = radialRes;
            this.angularRes = angularRes;
            
            paramsChanged.emit(direction, width, gain, range, radialRes, angularRes);
        }
        
        public void updateParams(int direction, int width, int gain, int range, int radialRes, int angularRes){
            AUV.this.controller.disable();
            this.setParams(direction, width, gain, range, radialRes, angularRes);
            AUV.this.controller.enable();
        }
    }
    
    public class Cameras {
        public final Camera FORWARD = new Camera(CameraID.Forward);
        public final Camera DOWNWARD = new Camera(CameraID.Down);
        public final Sonar SONAR = new Sonar(CameraID.Sonar);
    }

    public Cameras cameras = new Cameras();

   
    public class Script extends DataStream<String> {

        public Signal1<String> executionRequested = new Signal1<String>();
        public Signal1<String> responseReceieved = new Signal1<String>();

        public Script() {
            super("Scripting");
        }
        
        public void run(String mission){
            executionRequested.emit(mission);
        }
        
        public void update(String response){
            AUV.this.controller.disable();
            responseReceieved.emit(response);
            super.update(response);
            AUV.this.controller.enable();
        }
    }
    
    public class Scripting {
        public final Script CONSOLE = new Script();
        public final Script MISSION = new Script();
    }

    public Scripting scripting = new Scripting();
    
    /**
     * Logs any messages sent back from the AUV and also logs locally generated
     * error messages regarding the AUV, e.g. connection error messages
     * 
     * @author Andy Pritchard
     * 
     * @param <T>
     *            The type of the things being logged
     */
    public class Log<T> extends QSignalEmitter {

        public Vector<T> messages = new Vector<T>();

        public Signal1<String> messageLogged = new Signal1<String>();

        public void log(T message) {
            messages.add(message);
            messageLogged.emit(message.toString());
        }

        public Vector<T> getErrorMessages() {
            return messages;
        }
    }

    public class Logs {
        public Log<String> TRACE = new Log<String>();
        public Log<String> DEBUG = new Log<String>();
        public Log<String> ERROR = new Log<String>();
    }

    public Logs logs = new Logs();

    
    public class Orientation extends MultipleDataStreamEmitter {

        public MemorisingDataStream<Float> yawStream = new MemorisingDataStream<Float>("Yaw", 2000);
        public MemorisingDataStream<Float> pitchStream = new MemorisingDataStream<Float>("Pitch", 2000);
        public MemorisingDataStream<Float> rollStream = new MemorisingDataStream<Float>("Roll", 2000);

        public Signal1<Float> yawChanged = new Signal1<Float>();
        public Signal1<Float> pitchChanged = new Signal1<Float>();
        public Signal1<Float> rollChanged = new Signal1<Float>();
        public Signal1<floatYPR> orientationChanged = new Signal1<floatYPR>();
        
        public Orientation() {
            addStream(yawStream);
            addStream(pitchStream);
            addStream(rollStream);

            yawStream.latest = 0f;
            pitchStream.latest = 0f;
            rollStream.latest = 0f;
        }

        public Float getYaw() {
            return yawStream.latest;
        }
        
        public Float getPitch() {
            return pitchStream.latest;
        }
        
        public Float getRoll() {
            return rollStream.latest;
        }

        public floatYPR getOrientation(){
            floatYPR val =  new floatYPR();
            val.yaw = getYaw();
            val.pitch = getPitch();
            val.roll = getRoll();
            return val;
        }
        
        public void updateYaw(Float yaw) {
            AUV.this.controller.disable();
            yawStream.update(yaw);
            yawChanged.emit(yaw);
            orientationChanged.emit(getOrientation());
            AUV.this.controller.enable();
        }
        
        public void updatePitch(Float pitch) {
            AUV.this.controller.disable();
            pitchStream.update(pitch);
            pitchChanged.emit(pitch);
            orientationChanged.emit(getOrientation());
            AUV.this.controller.enable();
        }
        
        public void updateRoll(Float roll) {
            AUV.this.controller.disable();
            rollStream.update(roll);
            rollChanged.emit(roll);
            orientationChanged.emit(getOrientation());
            AUV.this.controller.enable();
        }
        
        public void updateOrientation(floatYPR orientation){
            this.updateYaw(orientation.yaw);
            this.updatePitch(orientation.pitch);
            this.updateRoll(orientation.roll);
        }
    }
    
    public class Depth extends MemorisingDataStream<Float> {

        public Signal1<Float> depthChanged = new Signal1<Float>();
        
        public Depth() {
            super("Depth", 2000);
            latest = 0f;
        }

        public Float getDepth() {
            return latest;
        }
        
        public void updateDepth(Float depth) {
            AUV.this.controller.disable();
            this.update(depth);
            depthChanged.emit(depth);
            AUV.this.controller.enable();
        }
    }
    
    public class Pressure extends MultipleDataStreamEmitter {

        public MemorisingDataStream<Integer> aftStream = new MemorisingDataStream<Integer>("Aft", 2000);
        public MemorisingDataStream<Integer> foreStream = new MemorisingDataStream<Integer>("Fore", 2000);
        
        public Pressure() {
            addStream(aftStream);
            addStream(foreStream);

            aftStream.latest = 0;
            foreStream.latest = 0;
        }

        public void updateFore(int fore){
            AUV.this.controller.disable();
            foreStream.update(fore);
            AUV.this.controller.enable();
        }

        public void updateAft(int aft){
            AUV.this.controller.disable();
            aftStream.update(aft);
            AUV.this.controller.enable();
        }
    }
    
    
    public class Telemetry {
        public Orientation ORIENTATION = new Orientation();
        public Depth DEPTH = new Depth();
        public Pressure PRESSURE = new Pressure();
    }
    
    public Telemetry telemetry = new Telemetry();
    
    /**
     * The AUV controller handles the messages sent over the network and also
     * monitors for updates to the AUV state, sending messages when necessary.
     */
    private CommunicationController controller;

    /**
     * Telemetry about the AUV that isn't categorised into a device above
     */
    public Signal4<Float, Float, Float, Float> depthCalibrationChanged = new Signal4<Float, Float, Float, Float>();
    public Signal1<Integer> debugLevelChanged = new Signal1<Integer>();
    public Signal0 resetMCB = new Signal0();
    public int debugLevel = 0;
    protected static Vector<AUVConnectionObserver> observers = new Vector<AUVConnectionObserver>();
    

    public AUV(String address, int port) throws UnknownHostException, IOException {
        controller = new CommunicationController(this, address, port);
        for(AUVConnectionObserver o: observers){
            o.onConnect(this);
        }
    }

    public CommunicationController getController() {
        return controller;
    }
    
    public static void registerAUVConnectionObserver(AUVConnectionObserver obs){
        observers.add(obs);
    }
    
    public void regsiterConnectionStateObserver(ConnectionStateObserver o) {
        controller.messages.addConnectionStateObserver(o);
    }

    public void resetMCB(){
        resetMCB.emit();
    }
    
    public void setDebugLevel(int level){
        this.debugLevel = level;
        debugLevelChanged.emit(level);
    }

    public void updateDebugLevel(int level){
        this.controller.disable();
        this.setDebugLevel(level);
        this.controller.enable();
    }

    public int getDebugLevel() {
        return debugLevel;
    }
        
    public void setDepthCalibration(float foreOffset, float foreScale, float aftOffset, float aftScale){
        depthCalibrationChanged.emit(foreOffset, foreScale, aftOffset, aftScale);
    }
        
    public void updateDepthCalibration(float foreOffset, float foreScale, float aftOffset, float aftScale){
        AUV.this.controller.disable();
        depthCalibrationChanged.emit(foreOffset, foreScale, aftOffset, aftScale);
        AUV.this.controller.enable();
    }
}
