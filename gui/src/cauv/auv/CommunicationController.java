package cauv.auv;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.net.UnknownHostException;

import cauv.auv.AUV.Autopilot;
import cauv.auv.AUV.Motor;
import cauv.auv.AUV.Sonar;
import cauv.messaging.BearingAutopilotEnabledMessage;
import cauv.messaging.BearingAutopilotParamsMessage;
import cauv.messaging.DebugLevelMessage;
import cauv.messaging.DebugMessage;
import cauv.messaging.DepthAutopilotEnabledMessage;
import cauv.messaging.DepthAutopilotParamsMessage;
import cauv.messaging.DepthCalibrationMessage;
import cauv.messaging.GuiImageMessage;
import cauv.messaging.InputStatusMessage;
import cauv.messaging.Message;
import cauv.messaging.MessageObserver;
import cauv.messaging.MotorMessage;
import cauv.messaging.ImageMessage;
import cauv.messaging.PitchAutopilotEnabledMessage;
import cauv.messaging.PitchAutopilotParamsMessage;
import cauv.messaging.PressureMessage;
import cauv.messaging.SonarControlMessage;
import cauv.messaging.TelemetryMessage;
import cauv.types.MotorID;



public class CommunicationController extends MessageObserver {
  
    protected AUV auv;
    protected MessageSocket messages;

    public MessageSocket getMessageSocket() {
        return messages;
    }
    
    class Controller {}
    
    class MissionController extends Controller {
        
        public MissionController() {
            auv.runMissionRequested.connect(this, "onMissionRunRequest(String)");
        }
        
        public void onMissionRunRequest(String mission){
            /*try {
                messages.sendMessage(new MissionMessage(mission));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error sending mission: " + e.getMessage());
            }*/
            System.out.println("Will send mission");
        }
    }
    
    class MotorController extends Controller {
        Motor motor;
        
        public MotorController(Motor m) {
            this.motor = m;
            m.speedChanged.connect(this, "onSpeedChanged(int)");
        }
        
        public void onSpeedChanged(int speed){
            try {
                messages.sendMessage(new MotorMessage(motor.getID(), (byte) speed));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error updating "+motor.getID().name()+": " + e.getMessage());
            }
        }
    }
    
    class SonarController extends Controller {
        Sonar sonar;
        
        public SonarController(Sonar s) {
            this.sonar = s;
            s.paramsChanged.connect(this, "onParamsChanged(int, int, int, int, int, int)");
        }
        
        public void onParamsChanged(int direction, int width, int gain, int range, int radialRes, int angularRes){
            try {
                messages.sendMessage(new SonarControlMessage(direction, width, gain, range, radialRes, angularRes));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error updating sonar: " + e.getMessage());
            }
        }
    }
    
    class AutopilotController extends Controller {
        Autopilot<?> autopilot;
        Class<?> enableMessage;
        Class<?> paramsMessage;
        
        
        public AutopilotController(Autopilot<?> a, Class<?> enableMessage, Class<?> paramsMessage) {
            this.autopilot = a;
            this.enableMessage = enableMessage;
            this.paramsMessage = paramsMessage;
            a.targetChanged.connect(this, "onTargetChanged(AUV$Autopilot)");
            a.stateChanged.connect(this, "onStateChanged(boolean)");
            a.paramsChanged.connect(this, "onParamsChanged(float, float, float, float)");
            
        }
        
        public void onStateChanged(boolean state){
            onTargetChanged(autopilot); 
        }
        
        public void onTargetChanged(Autopilot<?> autopilot){
            try {                
                Constructor c = enableMessage.getConstructor(Boolean.class, autopilot.target.getClass());
                Message m = (Message) c.newInstance(autopilot.enabled, autopilot.target);
                messages.sendMessage(m);
            } catch (Exception e) {
                e.printStackTrace();
                auv.logs.ERROR.log("Error updating autopilot: " + e.getMessage());
            }
        }
        
        public void onParamsChanged(float Kp, float Ki, float Kd, float scale){
            try {                
                Constructor c = paramsMessage.getConstructor(float.class, float.class, float.class, float.class);
                Message m = (Message) c.newInstance(autopilot.Kp, autopilot.Ki, autopilot.Kd, autopilot.scale);
                messages.sendMessage(m);
            } catch (Exception e) {
                auv.logs.ERROR.log("Error updating autopilot params: " + e.getMessage());
            }
        }
    }
    
    public CommunicationController(AUV auv, String address, int port) throws UnknownHostException,
            IOException {

        this.auv = auv;

        messages = new MessageSocket(address, port, "GUI");
        messages.joinGroup("control");
        messages.joinGroup("debug");
        messages.joinGroup("images");
        messages.joinGroup("telemetry");
        messages.joinGroup("sonarout");
        messages.joinGroup("pressure");

        new MotorController(auv.motors.HBOW);
        new MotorController(auv.motors.HSTERN);
        new MotorController(auv.motors.PROP);
        new MotorController(auv.motors.VBOW);
        new MotorController(auv.motors.VSTERN);
        
        new SonarController(auv.cameras.SONAR);
        
        new AutopilotController(auv.autopilots.DEPTH, DepthAutopilotEnabledMessage.class, DepthAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.PITCH, PitchAutopilotEnabledMessage.class, PitchAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.YAW, BearingAutopilotEnabledMessage.class, BearingAutopilotParamsMessage.class);
        
        new MissionController();
        
        auv.debugLevelChanged.connect(this, "sendDebugLevelMessage()");
        auv.depthCalibrationChanged.connect(this, "sendDepthCalibrationMessage(float, float)");
        
        enable();
        
        messages.addObserver(this);
    }

    public void enable() {
        messages.setEnabled(true);
    }

    public void disable() {
        messages.setEnabled(false);
    }
    
    public void sendDebugLevelMessage(){
        try {
            messages.sendMessage(new DebugLevelMessage(auv.getDebugLevel()));
        } catch (Exception e) {
            auv.logs.ERROR.log("Error updating debug level: " + e.getMessage());
        }
    }

    public void sendDepthCalibrationMessage(float fore, float aft){
        try {
            messages.sendMessage(new DepthCalibrationMessage(fore, aft));
        } catch (Exception e) {
            auv.logs.ERROR.log("Error updating depth calibration: " + e.getMessage());
        }
    }
    
    @Override
    public void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage m) {
        auv.autopilots.YAW.updateEnabled(m.enabled);
        auv.autopilots.YAW.updateTarget(m.target);
    }
    
    @Override
    public void onDebugLevelMessage(DebugLevelMessage m) {
        auv.updateDebugLevel(m.level);
    }
    
    @Override
    public void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage m) {
        auv.autopilots.YAW.updateParams(m.Kp, m.Ki, m.Kd, m.scale);
    }
    
    @Override
    public void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage m) {
        auv.autopilots.DEPTH.updateEnabled(m.enabled);
        auv.autopilots.DEPTH.updateTarget(m.target);
    }
    
    @Override
    public void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage m) {
        auv.autopilots.DEPTH.updateParams(m.Kp, m.Ki, m.Kd, m.scale);
    }
    
    @Override
    public void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage m) {
        auv.autopilots.PITCH.updateEnabled(m.enabled);
        auv.autopilots.PITCH.updateTarget(m.target);
    }
    
    @Override
    public void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage m) {
        auv.autopilots.PITCH.updateParams(m.Kp, m.Ki, m.Kd, m.scale);
    }
    
    @Override
    public void onSonarControlMessage(SonarControlMessage m) {
        auv.cameras.SONAR.updateParams(m.direction, m.width, m.gain, m.range, m.radialRes, m.angularRes);
    }
    
    @Override
    public void onDebugMessage(DebugMessage m) {
        switch(m.type){
            case Debug:
               auv.logs.DEBUG.log(m.msg);
                break;
            case Trace:
                auv.logs.TRACE.log(m.msg);
                break;
            case Error: 
                auv.logs.ERROR.log(m.msg);
                break;
        }
    }

    @Override
    public void onPressureMessage(PressureMessage m) {
        System.out.println(m);
        auv.updatePressures(m.fore, m.aft);
        auv.updateDepth((m.aft + m.fore)/2.0f);
    }
    
    @Override
    public void onTelemetryMessage(TelemetryMessage m) {
        auv.setOrientation(m.orientation);
    }
    
    @Override
    public void onImageMessage(ImageMessage m) {
        System.out.println(m);
        switch (m.source) {
            case Forward:
                auv.cameras.FORWARD.updateImage(m.image);
                break;
            case Down:
                auv.cameras.DOWNWARD.updateImage(m.image);
                break;
            case Sonar:
                auv.cameras.SONAR.updateImage(m.image);
                break;
        }
    }

    @Override
    public void onGuiImageMessage(GuiImageMessage m) {
        System.out.println(m);
    }
    
    
    @Override
    public void onMotorMessage(MotorMessage m) {
        switch (m.motorId) {
            case Prop:
                auv.motors.PROP.updateSpeed(m.speed);
                break;
            case HBow:
                auv.motors.HBOW.updateSpeed(m.speed);
                break;
            case HStern:
                auv.motors.HSTERN.updateSpeed(m.speed);
                break;
            case VBow:
                auv.motors.VBOW.updateSpeed(m.speed);
                break;
            case VStern:
                auv.motors.VSTERN.updateSpeed(m.speed);
                break;
        }
    }
}
