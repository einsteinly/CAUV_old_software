package cauv.auv;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.net.UnknownHostException;
import java.util.Vector;

import cauv.auv.AUV.Autopilot;
import cauv.auv.AUV.Motor;
import cauv.auv.AUV.Script;
import cauv.auv.AUV.Sonar;
import cauv.auv.MessageSocket.ConnectionStateObserver;
import cauv.messaging.BearingAutopilotEnabledMessage;
import cauv.messaging.BearingAutopilotParamsMessage;
import cauv.messaging.ControllerStateMessage;
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
import cauv.messaging.MotorStateMessage;
import cauv.messaging.PitchAutopilotEnabledMessage;
import cauv.messaging.PitchAutopilotParamsMessage;
import cauv.messaging.PressureMessage;
import cauv.messaging.ResetMCBMessage;
import cauv.messaging.ScriptMessage;
import cauv.messaging.ScriptResponseMessage;
import cauv.messaging.SonarControlMessage;
import cauv.messaging.TelemetryMessage;
import cauv.types.MotorID;



public class CommunicationController extends MessageObserver implements ConnectionStateObserver {

    protected AUV auv;
    protected MessageSocket messages;

    
    public interface AUVConnectionObserver {
        public void onConnect(AUV auv);
        public void onDisconnect();
    }
    
    abstract class Controller implements ConnectionStateObserver {
    }
    
    class ScriptController extends Controller {
        
        Script script;
        
        public ScriptController(Script s) {
            script = s;
        }
        
        public void onRunRequest(String mission){
            try {
                messages.sendMessage(new ScriptMessage(mission, 10f));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error sending script: " + e.getMessage());
            }
        }
        
        @Override
        public void onConnect(MessageSocket connection) {
            script.executionRequested.connect(this, "onRunRequest(String)");
        }
        
        @Override
        public void onDisconnect(MessageSocket connection) {
            script.executionRequested.disconnect(this, "onRunRequest(String)");
        }
    }
    
    class MotorController extends Controller {
        Motor motor;
        
        public MotorController(Motor m) {
            this.motor = m;
        }
        
        public void onSpeedChanged(int speed){
            try {
                messages.sendMessage(new MotorMessage(motor.getID(), (byte) speed));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error updating "+motor.getID().name()+": " + e.getMessage());
            }
        }
        
        @Override
        public void onConnect(MessageSocket connection) {
            motor.speedChanged.connect(this, "onSpeedChanged(int)");   
        }
        
        @Override
        public void onDisconnect(MessageSocket connection) {
            motor.speedChanged.disconnect(this, "onSpeedChanged(int)");
        }
    }
    
    class SonarController extends Controller {
        Sonar sonar;
        
        public SonarController(Sonar s) {
            this.sonar = s;
        }
        
        public void onParamsChanged(int direction, int width, int gain, int range, int radialRes, int angularRes){
            try {
                messages.sendMessage(new SonarControlMessage(direction, width, gain, range, radialRes, angularRes));
            } catch (IOException e) {
                auv.logs.ERROR.log("Error updating sonar: " + e.getMessage());
            }
        }
        
        @Override
        public void onConnect(MessageSocket connection) {
            sonar.paramsChanged.connect(this, "onParamsChanged(int, int, int, int, int, int)");
        }
        
        @Override
        public void onDisconnect(MessageSocket connection) {
            sonar.paramsChanged.disconnect(this, "onParamsChanged(int, int, int, int, int, int)");
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
        
        @Override
        public void onConnect(MessageSocket connection) {
            autopilot.targetChanged.connect(this, "onTargetChanged(AUV$Autopilot)");
            autopilot.enabledChanged.connect(this, "onStateChanged(boolean)");
            autopilot.paramsChanged.connect(this, "onParamsChanged()");
        }
        
        @Override
        public void onDisconnect(MessageSocket connection) {
            autopilot.targetChanged.disconnect(this, "onTargetChanged(AUV$Autopilot)");
            autopilot.enabledChanged.disconnect(this, "onStateChanged(boolean)");
            autopilot.paramsChanged.disconnect(this, "onParamsChanged()");
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
        messages.joinGroup("sonarctl");
        messages.joinGroup("pressure");
        messages.joinGroup("gui");

        messages.addConnectionStateObserver(this);
        
        messages.addConnectionStateObserver(new MotorController(auv.motors.HBOW));
        messages.addConnectionStateObserver(new MotorController(auv.motors.HSTERN));
        messages.addConnectionStateObserver(new MotorController(auv.motors.PROP));
        messages.addConnectionStateObserver(new MotorController(auv.motors.VBOW));
        messages.addConnectionStateObserver(new MotorController(auv.motors.VSTERN));
        
        messages.addConnectionStateObserver(new SonarController(auv.cameras.SONAR));
        
        messages.addConnectionStateObserver(new AutopilotController(auv.autopilots.DEPTH, DepthAutopilotEnabledMessage.class, DepthAutopilotParamsMessage.class));
        messages.addConnectionStateObserver(new AutopilotController(auv.autopilots.PITCH, PitchAutopilotEnabledMessage.class, PitchAutopilotParamsMessage.class));
        messages.addConnectionStateObserver(new AutopilotController(auv.autopilots.YAW, BearingAutopilotEnabledMessage.class, BearingAutopilotParamsMessage.class));

        messages.addConnectionStateObserver(new ScriptController(auv.scripting.CONSOLE));
        messages.addConnectionStateObserver(new ScriptController(auv.scripting.MISSION));
        
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

    public void sendResetMCBMessage(){
        try {
            messages.sendMessage(new ResetMCBMessage());
        } catch (Exception e) {
            auv.logs.ERROR.log("Error reseting MCB: " + e.getMessage());
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
    public void onDepthCalibrationMessage(DepthCalibrationMessage m) {
        auv.updateDepthCalibration(m.foreOffset(), m.foreMultiplier(), m.aftOffset(), m.aftMultiplier());
    }
    
    @Override
    public void onPressureMessage(PressureMessage m) {
        auv.telemetry.PRESSURE.aftStream.update(m.aft());
        auv.telemetry.PRESSURE.foreStream.update(m.fore());
    }
    
    @Override
    public void onTelemetryMessage(TelemetryMessage m) {
        auv.telemetry.DEPTH.updateDepth(m.depth());
        auv.telemetry.ORIENTATION.updateOrientation(m.orientation());
    }
    
    @Override
    public void onImageMessage(ImageMessage m) {
        switch (m.source()) {
            case Forward:
                auv.cameras.FORWARD.update(m.image());
                break;
            case Down:
                auv.cameras.DOWNWARD.update(m.image());
                break;
            case Sonar:
                auv.cameras.SONAR.update(m.image());
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

    @Override
    public void onConnect(MessageSocket connection) {
        auv.debugLevelChanged.connect(this, "sendDebugLevelMessage()");
        auv.resetMCB.connect(this, "sendResetMCBMessage()");
        auv.depthCalibrationChanged.connect(this, "sendDepthCalibrationMessage(float, float, float, float)");
    }

    @Override
    public void onDisconnect(MessageSocket connection) {
        auv.debugLevelChanged.disconnect(this, "sendDebugLevelMessage()");
        auv.resetMCB.disconnect(this, "sendResetMCBMessage()");
        auv.depthCalibrationChanged.disconnect(this, "sendDepthCalibrationMessage(float, float, float, float)");
        this.messages = null;
        
        auv.onDisconnect();
    }
}
