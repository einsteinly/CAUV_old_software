package cauv.auv;

import java.io.IOException;
import java.net.UnknownHostException;
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

    /**
     * Represents a motor in the AUV. All motors available are listed in the
     * Motors class Setting the motor speed (via setSpeed) will fire the
     * speedChanged signal to update interested objects. Calling the updateSpeed
     * method will disable the controller before setting the speed so that the
     * new motor speed is not sent to the AUV
     * 
     * @author Andy Pritchard
     */
    public class Motor extends QSignalEmitter {
        public Signal1<Integer> speedChanged = new Signal1<Integer>();

        protected int speed;
        protected MotorID id;

        public Motor(MotorID id) {
            this.id = id;
        }

        public MotorID getID() {
            return id;
        }

        public int getSpeed() {
            return speed;
        }

        public void setSpeed(int speed) {
            this.speed = speed;
            speedChanged.emit(speed);
        }

        protected void updateSpeed(int speed) {
            AUV.this.controller.disable();
            this.setSpeed(speed);
            AUV.this.controller.enable();
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
    	this.motors.HBOW.setSpeed(0);
    	this.motors.HSTERN.setSpeed(0);
    	this.motors.VBOW.setSpeed(0);
    	this.motors.VSTERN.setSpeed(0);
    	this.motors.PROP.setSpeed(0);

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
    public class Autopilot<T> extends QSignalEmitter {

        public Signal1<Autopilot<T>> targetChanged = new Signal1<Autopilot<T>>();
        public Signal1<Boolean> stateChanged = new Signal1<Boolean>();
        public Signal4<Float, Float, Float, Float> paramsChanged = new Signal4<Float, Float, Float, Float>();
        public Signal5<Float, Float, Float, Float, MotorDemand> controllerStateUpdated = new Signal5<Float, Float, Float, Float, MotorDemand>();

        protected T target;
        protected boolean enabled = true;
        protected float Kd, Kp, Ki;
        protected float scale;

        public Autopilot(T initialTarget) {
        	this.target = initialTarget;
		}
        
        public T getTarget() {
            return target;
        }

        public void setTarget(T target) {
            if (enabled) {
                if(!this.target.equals(target))
                {
                    this.target = target;
                    targetChanged.emit(this);
                }
            }
        }

        protected void updateTarget(T target) {
            AUV.this.controller.disable();
            this.setTarget(target);
            AUV.this.controller.enable();
        }
        
        public boolean getEnabled() {
            return enabled;
        }

        public void setEnabled(boolean state) {
            System.out.println("enabled " + state);
            this.enabled = state;
            stateChanged.emit(state);
        }

        protected void updateEnabled(boolean state) {
            AUV.this.controller.disable();
            this.setEnabled(state);
            AUV.this.controller.enable();
        }

        public float getKp() {
            return Kp;
        }
        
        public float getKi() {
            return Ki;
        }
        
        public float getKd() {
            return Kd;
        }

        public float getScale(){
            return scale;
        }
        
        public void setParams(float Kp, float Ki, float Kd, float scale) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.scale = scale;
            paramsChanged.emit(Kp, Ki, Kd, scale);
        }

        protected void updateParams(float Kp, float Ki, float Kd, float scale) {
            AUV.this.controller.disable();
            this.setParams(Kp, Ki, Kd, scale);
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
