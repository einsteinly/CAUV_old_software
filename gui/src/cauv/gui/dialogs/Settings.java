package cauv.gui.dialogs;

import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MessageSocket;
import cauv.auv.AUV.Autopilot;
import cauv.auv.CommunicationController.AUVConnectionObserver;
import cauv.types.floatYPR;

import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QAbstractSpinBox.CorrectionMode;

public class Settings extends QDialog implements AUVConnectionObserver {

    Ui_Settings ui = new Ui_Settings();
    AUV auv;
    
    
    public Settings() {
        ui.setupUi(this);
        
        ui.depth_Kp.returnPressed.connect(this, "updateDepthParams()");
        ui.depth_Ki.returnPressed.connect(this, "updateDepthParams()");
        ui.depth_Kd.returnPressed.connect(this, "updateDepthParams()");
        ui.depth_scale.returnPressed.connect(this, "updateDepthParams()");

        ui.pitch_Kp.returnPressed.connect(this, "updatePitchParams()");
        ui.pitch_Ki.returnPressed.connect(this, "updatePitchParams()");
        ui.pitch_Kd.returnPressed.connect(this, "updatePitchParams()");
        ui.pitch_scale.returnPressed.connect(this, "updatePitchParams()");

        ui.yaw_Kp.returnPressed.connect(this, "updateYawParams()");
        ui.yaw_Ki.returnPressed.connect(this, "updateYawParams()");
        ui.yaw_Kd.returnPressed.connect(this, "updateYawParams()");
        ui.yaw_scale.returnPressed.connect(this, "updateYawParams()");
        
        load();

        ui.debugMessaging.toggled.connect(this, "setDebug(boolean)");
        ui.gamepadID.valueChanged.connect(this, "save()");
    }

    public void save(){
        Config.GAMEPAD_ID = ui.gamepadID.value();
    }
    
    public void load(){
        ui.gamepadID.setValue(Config.GAMEPAD_ID);
    }
        
    public void updateDepthParamsUI(){
        ui.depth_Kp.setText(Float.toString(auv.autopilots.DEPTH.getKp()));
        ui.depth_Ki.setText(Float.toString(auv.autopilots.DEPTH.getKi()));
        ui.depth_Kd.setText(Float.toString(auv.autopilots.DEPTH.getKd()));
        ui.depth_scale.setText(Float.toString(auv.autopilots.DEPTH.getScale()));
    }
    
    public void updateDepthParams(){
        if(auv == null) return;
        auv.autopilots.DEPTH.setParams(Float.valueOf(ui.depth_Kp.text()),
                Float.valueOf(ui.depth_Ki.text()),
                Float.valueOf(ui.depth_Kd.text()),
                Float.valueOf(ui.depth_scale.text()));
    }
    
    public void updatePitchParamsUI(){
        ui.pitch_Kp.setText(Float.toString(auv.autopilots.PITCH.getKp()));
        ui.pitch_Ki.setText(Float.toString(auv.autopilots.PITCH.getKi()));
        ui.pitch_Kd.setText(Float.toString(auv.autopilots.PITCH.getKd()));
        ui.pitch_scale.setText(Float.toString(auv.autopilots.PITCH.getScale()));
    }
    
    public void updatePitchParams(){
        if(auv == null) return;
        auv.autopilots.PITCH.setParams(Float.valueOf(ui.pitch_Kp.text()),
                Float.valueOf(ui.pitch_Ki.text()),
                Float.valueOf(ui.pitch_Kd.text()),
                Float.valueOf(ui.pitch_scale.text()));
    }  
    
    public void updateYawParamsUI(){
        ui.yaw_Kp.setText(Float.toString(auv.autopilots.YAW.getKp()));
        ui.yaw_Ki.setText(Float.toString(auv.autopilots.YAW.getKi()));
        ui.yaw_Kd.setText(Float.toString(auv.autopilots.YAW.getKd()));
        ui.yaw_scale.setText(Float.toString(auv.autopilots.YAW.getScale()));
    }
    
    public void updateYawParams(){
        System.out.println("updating yaw");
        if(auv == null) return;
        auv.autopilots.YAW.setParams(Float.valueOf(ui.yaw_Kp.text()),
                Float.valueOf(ui.yaw_Ki.text()),
                Float.valueOf(ui.yaw_Kd.text()),
                Float.valueOf(ui.yaw_scale.text()));
    }
    
    public void onConnect(AUV auv){
        this.auv = auv;
        
        auv.autopilots.DEPTH.paramsChanged.connect(this, "updateDepthParamsUI()");
        auv.autopilots.PITCH.paramsChanged.connect(this, "updatePitchParamsUI()");
        auv.autopilots.YAW.paramsChanged.connect(this, "updateYawParamsUI()");
        
        auv.debugLevelChanged.connect(ui.debugLevel, "setValue(int)");
        ui.debugLevel.valueChanged.connect(auv, "setDebugLevel(int)");
        
        ui.aftOffset.valueChanged.connect(this, "calibrateDepth()");
        ui.aftScale.valueChanged.connect(this, "calibrateDepth()");
        ui.foreOffset.valueChanged.connect(this, "calibrateDepth()");
        ui.foreScale.valueChanged.connect(this, "calibrateDepth()");

        ui.depthEnabled.toggled.connect(auv.autopilots.DEPTH, "setEnabled(boolean)");
        auv.autopilots.DEPTH.enabledChanged.connect(ui.depthEnabled, "setChecked(boolean)");
        ui.depthTarget.returnPressed.connect(this, "updateDepthTarget()");
        auv.autopilots.DEPTH.targetChanged.connect(this, "updateDepthTarget(AUV$Autopilot)");
        

        ui.yawEnabled.toggled.connect(auv.autopilots.YAW, "setEnabled(boolean)");
        auv.autopilots.YAW.enabledChanged.connect(ui.yawEnabled, "setChecked(boolean)");
        ui.yawTarget.returnPressed.connect(this, "updateYawTarget()");
        auv.autopilots.YAW.targetChanged.connect(this, "updateYawTarget(AUV$Autopilot)");
        

        ui.pitchEnabled.toggled.connect(auv.autopilots.PITCH, "setEnabled(boolean)");
        auv.autopilots.PITCH.enabledChanged.connect(ui.pitchEnabled, "setChecked(boolean)");
        ui.pitchTarget.returnPressed.connect(this, "updatePitchTarget()");
        auv.autopilots.PITCH.targetChanged.connect(this, "updatePitchTarget(AUV$Autopilot)");

        auv.depthChanged.connect(this, "updateDepthActual(float)");
        auv.orientationChanged.connect(this, "updateOrientationActual(floatYPR)");
    }
    
    public void onDisconnect() {
       auv = null;
    }
    
    public void updateDepthActual(float depth){
        ui.depthActual.setText("Actual: " + depth);
    }
    
    public void updateDepthTarget(Autopilot<Float> f){
        ui.depthTarget.setText(f.getTarget().toString());
    }
    
    public void updateDepthTarget(){
        auv.autopilots.DEPTH.setTarget(Float.valueOf(ui.depthTarget.text()));
    }
    
    public void updateYawTarget(Autopilot<Float> f){
        ui.yawTarget.setText(f.getTarget().toString());
    }
    
    public void updateYawTarget(){
        auv.autopilots.YAW.setTarget(Float.valueOf(ui.yawTarget.text()));
    }
    
    public void updateOrientationActual(floatYPR orientation){
        ui.pitchActual.setText("Actual: " + orientation.pitch);
        ui.yawActual.setText("Actual: " + orientation.yaw);
    }
    
    public void updatePitchTarget(Autopilot<Float> f){
        ui.pitchTarget.setText(f.getTarget().toString());
    }
    
    public void updatePitchTarget(){
        auv.autopilots.PITCH.setTarget(Float.valueOf(ui.pitchTarget.text()));
    }
    
    public void setDebug(boolean state){
        MessageSocket.setDebug(state);
    }
    
    public void calibrateDepth(){
        auv.calibrateDepth((float)ui.foreOffset.value(), (float)ui.foreScale.value(),
                (float)ui.aftOffset.value(), (float)ui.aftScale.value());
    }
}
