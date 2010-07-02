package cauv.auv;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.net.UnknownHostException;
import java.util.Vector;

import cauv.auv.AUV.Autopilot;
import cauv.auv.AUV.Motor;
import cauv.auv.AUV.Script;
import cauv.auv.AUV.Sonar;
import cauv.messaging.BearingAutopilotEnabledMessage;
import cauv.messaging.BearingAutopilotParamsMessage;
import cauv.messaging.ControllerStateMessage;
import cauv.messaging.DebugLevelMessage;
import cauv.messaging.DebugMessage;
import cauv.messaging.DepthAutopilotEnabledMessage;
import cauv.messaging.DepthAutopilotParamsMessage;
import cauv.messaging.DepthCalibrationMessage;
import cauv.messaging.DepthMessage;
import cauv.messaging.GuiImageMessage;
import cauv.messaging.InputStatusMessage;
import cauv.messaging.Message;
import cauv.messaging.MessageObserver;
import cauv.messaging.MotorMessage;
import cauv.messaging.ImageMessage;
import cauv.messaging.MotorStateMessage;
import cauv.messaging.PitchAutopilotEnabledMessage;
import cauv.messaging.PitchAutopilotParamsMessage;
import cauv.messaging.PressureMessage;
import cauv.messaging.ScriptMessage;
import cauv.messaging.ScriptResponseMessage;
import cauv.messaging.SonarControlMessage;
import cauv.messaging.TelemetryMessage;
import cauv.types.MotorID;



public class CommunicationController extends MessageObserver {

    protected AUV auv;
    protected MessageSocket messages;

    public MessageSocket getMessageSocket() {
        return messages;
    }

    
    public interface AUVConnectionObserver {
        public void onConnect(AUV auv);
        public void onDisconnect();
    }
    
    class Controller {}
    
    class ScriptController extends Controller {
        
        public ScriptController(Script s) {
            s.executionRequested.connect(this, "onRunRequest(String)");
        }
        
        public void onRunRequest(String mission){
            try {
                messages.sendMessage(new ScriptMessage(mission, 10f));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error sending script: " + e.getMessage());
            }
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
            a.enabledChanged.connect(this, "onStateChanged(boolean)");
            a.paramsChanged.connect(this, "onParamsChanged()");
            
        }
        
        public void onStateChanged(boolean state){
            onTargetChanged(autopilot); 
        }
        
        public void onTargetChanged(Autopilot<?> autopilot){
            try {                
                Constructor c = enableMessage.getConstructor(Boolean.class, autopilot.getTarget().getClass());
                Message m = (Message) c.newInstance(autopilot.getEnabled(), autopilot.getTarget());
                messages.sendMessage(m);
            } catch (Exception e) {
                e.printStackTrace();
                auv.logs.ERROR.log("Error updating autopilot: " + e.getMessage());
            }
        }
        
        public void onParamsChanged(){
            try {                
                Constructor c = paramsMessage.getConstructor(Float.class, Float.class, Float.class, Float.class);
                Message m = (Message) c.newInstance(autopilot.getKp(), autopilot.getKi(), autopilot.getKd(), autopilot.getScale());
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
        messages.joinGroup("image");
        messages.joinGroup("telemetry");
        messages.joinGroup("sonarout");
        messages.joinGroup("pressure");
        messages.joinGroup("gui");

        new MotorController(auv.motors.HBOW);
        new MotorController(auv.motors.HSTERN);
        new MotorController(auv.motors.PROP);
        new MotorController(auv.motors.VBOW);
        new MotorController(auv.motors.VSTERN);
        
        new SonarController(auv.cameras.SONAR);
        
        new AutopilotController(auv.autopilots.DEPTH, DepthAutopilotEnabledMessage.class, DepthAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.PITCH, PitchAutopilotEnabledMessage.class, PitchAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.YAW, BearingAutopilotEnabledMessage.class, BearingAutopilotParamsMessage.class);

        new ScriptController(auv.scripting.CONSOLE);
        new ScriptController(auv.scripting.MISSION);
        
        auv.debugLevelChanged.connect(this, "sendDebugLevelMessage()");
        auv.depthCalibrationChanged.connect(this, "sendDepthCalibrationMessage(float, float, float, float)");
        
        enable();
        
        messages.addObserver(this);
    }

    public boolean getEnabled(){
        return messages.enabled;
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

    public void sendDepthCalibrationMessage( float foreOffset, float foreScale, float aftOffset, float aftScale){
        try {
            messages.sendMessage(new DepthCalibrationMessage(foreOffset, foreScale, aftOffset, aftScale));
        } catch (Exception e) {
            auv.logs.ERROR.log("Error updating depth calibration: " + e.getMessage());
        }
    }
    
    @Override
    public void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage m) {
        auv.autopilots.YAW.updateEnabled(m.enabled());
        auv.autopilots.YAW.updateTarget(m.target());
    }
    
    @Override
    public void onDebugLevelMessage(DebugLevelMessage m) {
        auv.updateDebugLevel(m.level());
    }
    
    @Override
    public void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage m) {
        auv.autopilots.YAW.updateParams(m.Kp(), m.Ki(), m.Kd(), m.scale());
    }
    
    @Override
    public void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage m) {
        auv.autopilots.DEPTH.updateEnabled(m.enabled());
        auv.autopilots.DEPTH.updateTarget(m.target());
    }
    
    @Override
    public void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage m) {
        System.out.println(m);
        auv.autopilots.DEPTH.updateParams(m.Kp(), m.Ki(), m.Kd(), m.scale());
    }
    
    @Override
    public void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage m) {
        auv.autopilots.PITCH.updateEnabled(m.enabled());
        auv.autopilots.PITCH.updateTarget(m.target());
    }
    
    @Override
    public void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage m) {
        auv.autopilots.PITCH.updateParams(m.Kp(), m.Ki(), m.Kd(), m.scale());
    }
    
    @Override
    public void onSonarControlMessage(SonarControlMessage m) {
        auv.cameras.SONAR.updateParams(m.direction(), m.width(), m.gain(), m.range(), m.radialRes(), m.angularRes());
    }
    
    @Override
    public void onControllerStateMessage(ControllerStateMessage m) {
        cauv.types.Controller c = m.contoller();
        switch(c){
            case Bearing:
                auv.autopilots.YAW.updateControllerState(m.mv(), m.error(), m.derror(), m.ierror(), m.demand());
                break;
            case Pitch:
                auv.autopilots.PITCH.updateControllerState(m.mv(), m.error(), m.derror(), m.ierror(), m.demand());
                break;
            case Depth:
                auv.autopilots.DEPTH.updateControllerState(m.mv(), m.error(), m.derror(), m.ierror(), m.demand());
                break;
        }
    }
    
    @Override
    public void onMotorStateMessage(MotorStateMessage m) {
    }
    
    
    
    @Override
    public void onScriptResponseMessage(ScriptResponseMessage m) {
        auv.scripting.CONSOLE.responseReceieved.emit(m.response());
    }
    
    @Override
    public void onDebugMessage(DebugMessage m) {
        System.out.println(m);
        switch(m.type()){
            case Debug:
               auv.logs.DEBUG.log(m.msg());
                break;
            case Trace:
                auv.logs.TRACE.log(m.msg());
                break;
            case Error: 
                auv.logs.ERROR.log(m.msg());
                break;
        }
    }

    @Override
    public void onDepthMessage(DepthMessage m) {
        auv.updateDepth(m.depth());
    }
    
    @Override
    public void onPressureMessage(PressureMessage m) {
        auv.updatePressures(m.fore(), m.aft());
    }
    
    @Override
    public void onTelemetryMessage(TelemetryMessage m) {
        auv.setOrientation(m.orientation());
    }
    
    @Override
    public void onImageMessage(ImageMessage m) {
        switch (m.source()) {
            case Forward:
                auv.cameras.FORWARD.updateImage(m.image());
                break;
            case Down:
                auv.cameras.DOWNWARD.updateImage(m.image());
                break;
            case Sonar:
                auv.cameras.SONAR.updateImage(m.image());
                break;
        }
    }

    @Override
    public void onGuiImageMessage(GuiImageMessage m) {
        System.out.println("on gui image message");
        System.out.println(m);
    }
    
    
    @Override
    public void onMotorMessage(MotorMessage m) {
        switch (m.motorId()) {
            case Prop:
                auv.motors.PROP.update((int)m.speed());
                break;
            case HBow:
                auv.motors.HBOW.update((int)m.speed());
                break;
            case HStern:
                auv.motors.HSTERN.update((int)m.speed());
                break;
            case VBow:
                auv.motors.VBOW.update((int)m.speed());
                break;
            case VStern:
                auv.motors.VSTERN.update((int)m.speed());
                break;
        }
    }
}
