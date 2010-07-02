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

    
    public class DataStream<T> extends QSignalEmitter {
        
        public Signal1<DataStream<T>> onChange = new Signal1<DataStream<T>>();
        
        T latest;
        
        public void update(T data){
            boolean state = AUV.this.controller.getEnabled();
            AUV.this.controller.disable();
            this.set(data);
            if(state)
                AUV.this.controller.enable();
        }
        
        public void set(T data) {
            this.onChange.emit(this);
            this.latest = data;
        }
        
        public T get() {
            return latest;
        }
    }
    
    public class MemorisingDataStream<T> extends DataStream<T> {
        Vector<T> history = new Vector<T>();
        
        int maximum;
        
        public MemorisingDataStream(int maximum) {
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
        
        protected void addStream(String name, DataStream<?> data){
            this.dataStreams.put(name, data);
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
            this.id = id;
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

        
        public MemorisingDataStream<MotorDemand> demandsStream = new MemorisingDataStream<MotorDemand>(5000);
        public DataStream<T> targetStream = new DataStream<T>();
        public DataStream<Boolean> enabledStream = new DataStream<Boolean>();
        public MemorisingDataStream<Float> KpStream = new MemorisingDataStream<Float>(5000);
        public MemorisingDataStream<Float> KiStream = new MemorisingDataStream<Float>(5000);
        public MemorisingDataStream<Float> KdStream = new MemorisingDataStream<Float>(5000);
        public MemorisingDataStream<Float> scaleStream = new MemorisingDataStream<Float>(5000);
        
        public Signal1<Autopilot<T>> targetChanged = new Signal1<Autopilot<T>>();
        public Signal1<Boolean> enabledChanged = new Signal1<Boolean>();
        public Signal0 paramsChanged = new Signal0();
        
        public Autopilot(T initialTarget) {
            addStream("demands", demandsStream);
            addStream("target", targetStream);
            addStream("enabled", enabledStream);
            addStream("Kp", KpStream);
            addStream("Ki", KiStream);
            addStream("Kd", KdStream);
            addStream("scale", scaleStream);
            
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

        protected void updateTarget(T target) {
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

        protected void updateEnabled(boolean state) {
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
        }
        
        public void setParams(float Kp, float Ki, float Kd, float scale) {
            KpStream.set(Kp);
            KiStream.set(Ki);
            KdStream.set(Kd);
            scaleStream.set(scale);
            paramsChanged.emit();
        }

        protected void updateParams(float Kp, float Ki, float Kd, float scale) {
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
        public final Autopilot<Float> DEPTH = new Autopilot<Float>(0.0f);
        public final Autopilot<Float> YAW = new Autopilot<Float>(0.0f);
        public final Autopilot<Float> PITCH = new Autopilot<Float>(0.0f);
    }

    public Autopilots autopilots = new Autopilots();

    /**
     * Represents a camera in the AUV. When a new image is received the
     * imageReceived signal is emitted. Images cannot be sent to the AUV
     * 
     * @author Andy Pritchard
     * 
     */
    public class Camera extends QSignalEmitter {

        //public Signal1<Image> imageReceived = new Signal1<Image>();

        protected CameraID id;

        public Image lastImage;

        public Camera(CameraID id) {
            this.id = id;
        }

        public Image getLastImage() {
            return lastImage;
        }

        protected void updateImage(Image image) {
            this.lastImage = image;
            //imageReceived.emit(image);
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

   
    public class Script extends QSignalEmitter {

        public Signal1<String> executionRequested = new Signal1<String>();
        public Signal1<String> responseReceieved = new Signal1<String>();

        public Script() {
        }
        
        public void run(String mission){
            executionRequested.emit(mission);
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

    /**
     * The AUV controller handles the messages sent over the network and also
     * monitors for updates to the AUV state, sending messages when necessary.
     */
    private CommunicationController controller;

    /**
     * Telemetry about the AUV that isn't categorised into a device above
     */
    public Signal1<floatYPR> orientationChanged = new Signal1<floatYPR>();
    protected floatYPR orientation = new floatYPR();
    public Signal1<Float> depthChanged = new Signal1<Float>();
    public Signal4<Float, Float, Float, Float> depthCalibrationChanged = new Signal4<Float, Float, Float, Float>();
    public Signal2<Float, Float> pressureChanged = new Signal2<Float, Float>();
    public Signal1<Integer> debugLevelChanged = new Signal1<Integer>();
    public int debugLevel = 0;
    protected float depth = 0.0f;
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

    protected void setOrientation(floatYPR orientation) {
        this.orientation = orientation;
        orientationChanged.emit(orientation);
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

    public void updatePressures(float fore, float aft){
        this.controller.disable();
        pressureChanged.emit(fore, aft);
        this.controller.enable();
    }
    
    public int getDebugLevel() {
        return debugLevel;
    }
    
    public floatYPR getOrientation() {
        return orientation;
    }
    
    public void setDepth(float depth) {
        this.depth = depth;
        depthChanged.emit(depth);
    }
    
    public void calibrateDepth(float foreOffset, float foreScale, float aftOffset, float aftScale){
        depthCalibrationChanged.emit(foreOffset, foreScale, aftOffset, aftScale);
    }

    public void updateDepth(float depth){
        this.controller.disable();
        this.setDepth(depth);
        this.controller.enable();
    }
    
    public float getDepth() {
        return depth;
    }
}
