package cauv.auv;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.net.UnknownHostException;

import cauv.auv.AUV.Autopilot;
import cauv.auv.AUV.Motor;
import cauv.messaging.BearingAutopilotEnabledMessage;
import cauv.messaging.BearingAutopilotParamsMessage;
import cauv.messaging.DebugMessage;
import cauv.messaging.DepthAutopilotEnabledMessage;
import cauv.messaging.DepthAutopilotParamsMessage;
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



class CommunicationController extends MessageObserver {
  
    protected AUV auv;
    protected MessageSocket messages;

    
    class Controller {}
    
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
    
    class AutopilotController extends Controller {
        Autopilot<?> autpilot;
        Class<?> enableMessage;
        Class<?> paramsMessage;
        
        
        public AutopilotController(Autopilot<?> a, Class<?> enableMessage, Class<?> paramsMessage) {
            this.autpilot = a;
            this.enableMessage = enableMessage;
            this.paramsMessage = paramsMessage;
            a.targetChanged.connect(this, "onTargetChanged(AUV$Autopilot)");
            
        }
        
        public void onTargetChanged(Autopilot<?> autopilot){
            try {
                for(Constructor c : enableMessage.getConstructors())
                    System.out.println(c);
                
                Constructor c = enableMessage.getConstructor(boolean.class, autopilot.target.getClass());
                Message m = (Message) c.newInstance(autopilot.enabled, autopilot.target);
                messages.sendMessage(m);
            } catch (Exception e) {
                e.printStackTrace();
                //auv.logs.ERROR.log("Error updating autopilot: " + e.getMessage());
            }
        }
    }
    
    public CommunicationController(AUV auv, String address, int port) throws UnknownHostException,
            IOException {

        this.auv = auv;

        messages = new MessageSocket(address, port, "GUI");
        messages.addObserver(this);
        messages.joinGroup("control");
        messages.joinGroup("images");
        messages.joinGroup("trace");

        new MotorController(auv.motors.HBOW);
        new MotorController(auv.motors.HSTERN);
        new MotorController(auv.motors.PROP);
        new MotorController(auv.motors.VBOW);
        new MotorController(auv.motors.VSTERN);
        
        
        new AutopilotController(auv.autopilots.DEPTH, DepthAutopilotEnabledMessage.class, DepthAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.PITCH, PitchAutopilotEnabledMessage.class, PitchAutopilotParamsMessage.class);
        new AutopilotController(auv.autopilots.YAW, BearingAutopilotEnabledMessage.class, BearingAutopilotParamsMessage.class);
        
        enable();
    }

    public void enable() {
        
        messages.setEnabled(true);
        /*
        // autopilots
        auv.autopilots.DEPTH.targetChanged.connect(this, "updateDepthAutopilot(AUV$Autopilot)");
        auv.autopilots.DEPTH.stateChanged.connect(this, "updateDepthAutopilot(AUV$Autopilot)");
        auv.autopilots.DEPTH.paramsChanged.connect(this, "updateDepthAutopilotParams(float, float, float, float)");
        
        auv.autopilots.PITCH.targetChanged.connect(this, "updatePitchAutopilot(AUV$Autopilot)");
        auv.autopilots.YAW.targetChanged.connect(this, "updateYawAutopilot(AUV$Autopilot)");
        //*/
    }

    public void disable() {
        messages.setEnabled(false);
    }

    protected void updateYawAutopilot(Autopilot<Float> autopilot) {
        try {
            messages.sendMessage(new BearingAutopilotEnabledMessage(autopilot.enabled, autopilot.target));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating yaw autopilot: " + e.getMessage());
        }
    }
    
    protected void updateDepthAutopilot(Autopilot<Float> autopilot) {
        try {
            messages.sendMessage(new DepthAutopilotEnabledMessage(autopilot.enabled, autopilot.target));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating depth autopilot: " + e.getMessage());
        }
    } 
    
    protected void updatePitchAutopilot(Autopilot<Float> autopilot) {
        try {
            messages.sendMessage(new PitchAutopilotEnabledMessage(autopilot.enabled, autopilot.target));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating pitch autopilot: " + e.getMessage());
        }
    }

    protected void updateHBow(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.HBow, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating HBOW: " + e.getMessage());
        }
    }

    protected void updateVBow(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.VBow, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating VBOW: " + e.getMessage());
        }
    }

    protected void updateHStern(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.HStern, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating HSTERN: " + e.getMessage());
        }
    }

    protected void updateVStern(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.VStern, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating VSTERN: " + e.getMessage());
        }
    }

    protected void updateProp(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.Prop, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating PROP: " + e.getMessage());
        }
    }

    @Override
    public void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage m) {
        auv.autopilots.YAW.updateEnabled(m.enabled);
        auv.autopilots.YAW.updateTarget(m.target);
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
        auv.setDepth((m.aft + m.fore)/2);
    }
    
    @Override
    public void onTelemetryMessage(TelemetryMessage m) {
        auv.setOrientation(m.orientation);
    }
    
    @Override
    public void onImageMessage(ImageMessage m) {
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
