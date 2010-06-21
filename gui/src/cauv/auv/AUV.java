package cauv.auv;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Vector;

import cauv.auv.MessageSocket.ConnectionStateObserver;
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
        public final Motor PROP = new Motor(MotorID.PROP);
        public final Motor HBOW = new Motor(MotorID.HBOW);
        public final Motor VBOW = new Motor(MotorID.VBOW);
        public final Motor HSTERN = new Motor(MotorID.HSTERN);
        public final Motor VSTERN = new Motor(MotorID.VSTERN);
    }

    public Motors motors = new Motors();

    
    public void stopAllMotors(){
    	this.motors.HBOW.setSpeed(0);
    	this.motors.HSTERN.setSpeed(0);
    	this.motors.VBOW.setSpeed(0);
    	this.motors.VSTERN.setSpeed(0);
    	this.motors.PROP.setSpeed(0);
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

        protected T target;
        protected boolean enabled = true;

        public Autopilot(T initialTarget) {
        	this.target = initialTarget;
		}
        
        public T getTarget() {
            return target;
        }

        public void setTarget(T target) {
            if (enabled) {
                this.target = target;
                targetChanged.emit(this);
            }
        }

        public boolean getEnabled() {
            return enabled;
        }

        public void setEnabled(boolean state) {
            this.enabled = state;
            stateChanged.emit(state);
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
    public static class Camera extends QSignalEmitter {

        public Signal1<Vector<Byte>> imageReceived = new Signal1<Vector<Byte>>();

        protected CameraID id;

        public Vector<Byte> lastImage;

        public Camera(CameraID id) {
            this.id = id;
        }

        public Vector<Byte> getLastImage() {
            return lastImage;
        }

        protected void updateImage(Vector<Byte> image) {
            this.lastImage = image;
            imageReceived.emit(image);
        }
    }

    public class Cameras {
        public final Camera FORWARD = new Camera(CameraID.FORWARD);
        public final Camera DOWNWARD = new Camera(CameraID.DOWN);
        public final Camera SONAR = new Camera(CameraID.SONAR);
    }

    public Cameras cameras = new Cameras();

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

        public Signal1<T> messageLogged = new Signal1<T>();

        public void log(T message) {
            messages.add(message);
            messageLogged.emit(message);
        }

        public Vector<T> getErrorMessages() {
            return messages;
        }
    }

    public class Logs {
        public Log<String> TRACE = new Log<String>();
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
    protected float depth = 0.0f;

    public AUV(String address, int port) throws UnknownHostException, IOException {
        controller = new CommunicationController(this, address, port);
    }

    public void regsiterConnectionStateObserver(ConnectionStateObserver o) {
        controller.messages.addConnectionStateObserver(o);
    }

    protected void setOrientation(floatYPR orientation) {
        this.orientation = orientation;
        orientationChanged.emit(orientation);
    }

    public floatYPR getOrientation() {
        return orientation;
    }
    
    protected void setDepth(float depth) {
        this.depth = depth;
        depthChanged.emit(depth);
    }

    public float getDepth() {
        return depth;
    }
}
