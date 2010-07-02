package cauv.gui.views;

import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MessageSocket;
import cauv.gui.ScreenView;
import cauv.messaging.MessageSource;
import cauv.types.floatYPR;

import com.trolltech.qt.core.Qt.ConnectionType;
import com.trolltech.qt.gui.*;

public class SettingsView extends QWidget implements ScreenView {

    AUV auv;
    
	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_SettingsView ui = new Ui_SettingsView();
	
    public SettingsView() {
        ui.setupUi(this);
		icon.setPixmap(new QPixmap("classpath:cauv/gui/resources/settings.png"));

        ui.depth_Kp.valueChanged.connect(this, "updateDepthParams()");
        ui.depth_Ki.valueChanged.connect(this, "updateDepthParams()");
        ui.depth_Kd.valueChanged.connect(this, "updateDepthParams()");
        ui.depth_scale.valueChanged.connect(this, "updateDepthParams()");

        ui.pitch_Kp.valueChanged.connect(this, "updatePitchParams()");
        ui.pitch_Ki.valueChanged.connect(this, "updatePitchParams()");
        ui.pitch_Kd.valueChanged.connect(this, "updatePitchParams()");
        ui.pitch_scale.valueChanged.connect(this, "updatePitchParams()");

        ui.yaw_Kp.valueChanged.connect(this, "updateYawParams()");
        ui.yaw_Ki.valueChanged.connect(this, "updateYawParams()");
        ui.yaw_Kd.valueChanged.connect(this, "updateYawParams()");
        ui.yaw_scale.valueChanged.connect(this, "updateYawParams()");
        
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
        
    public void updateDepthParams(float Kp, float Ki, float Kd, float scale){
        ui.depth_Kp.setValue(Kp);
        ui.depth_Ki.setValue(Ki);
        ui.depth_Kd.setValue(Kd);
        ui.depth_scale.setValue(scale);
    }
    
    public void updateDepthParams(){
        if(auv == null) return;
        auv.autopilots.DEPTH.setParams((float)ui.depth_Kp.value(),
                (float) ui.depth_Ki.value(),
                (float) ui.depth_Kd.value(),
                (float) ui.depth_scale.value());
    }
    
    public void updatePitchParams(float Kp, float Ki, float Kd, float scale){
        ui.pitch_Kp.setValue(Kp);
        ui.pitch_Ki.setValue(Ki);
        ui.pitch_Kd.setValue(Kd);
        ui.pitch_scale.setValue(scale);
    }
    
    public void updatePitchParams(){
        if(auv == null) return;
        auv.autopilots.PITCH.setParams((float)ui.pitch_Kp.value(),
                (float) ui.pitch_Ki.value(),
                (float) ui.pitch_Kd.value(),
                (float) ui.pitch_scale.value());
    }  
    
    public void updateYawParams(float Kp, float Ki, float Kd, float scale){
        ui.yaw_Kp.setValue(Kp);
        ui.yaw_Ki.setValue(Ki);
        ui.yaw_Kd.setValue(Kd);
        ui.yaw_scale.setValue(scale);
    }
    
    public void updateYawParams(){
        if(auv == null) return;
        auv.autopilots.YAW.setParams((float)ui.yaw_Kp.value(),
                (float) ui.yaw_Ki.value(),
                (float) ui.yaw_Kd.value(),
                (float) ui.yaw_scale.value());
    }
    
    public void onConnect(AUV auv){
        this.auv = auv;
        
        auv.autopilots.DEPTH.paramsChanged.connect(this, "updateDepthParams(float, float, float, float)");
        auv.autopilots.PITCH.paramsChanged.connect(this, "updatePitchParams(float, float, float, float)");
        auv.autopilots.YAW.paramsChanged.connect(this, "updateYawParams(float, float, float, float)");
        
        auv.debugLevelChanged.connect(ui.debugLevel, "setValue(int)");
        ui.debugLevel.valueChanged.connect(auv, "setDebugLevel(int)");
        
        ui.aftOffset.valueChanged.connect(this, "calibrateDepth()");
        ui.aftScale.valueChanged.connect(this, "calibrateDepth()");
        ui.foreOffset.valueChanged.connect(this, "calibrateDepth()");
        ui.foreScale.valueChanged.connect(this, "calibrateDepth()");

        ui.depthEnabled.toggled.connect(auv.autopilots.DEPTH, "setEnabled(boolean)");
        auv.autopilots.DEPTH.enabledChanged.connect(ui.depthEnabled, "setChecked(boolean)");
        ui.depthValue.valueChanged.connect(this, "updateDepthTarget(double)");
        auv.autopilots.DEPTH.targetChanged.connect(this, "updateDepthTarget()");
        

        ui.yawEnabled.toggled.connect(auv.autopilots.YAW, "setEnabled(boolean)");
        auv.autopilots.YAW.enabledChanged.connect(ui.yawEnabled, "setChecked(boolean)");
        ui.yawTarget.valueChanged.connect(this, "updateYawTarget(double)");
        auv.autopilots.YAW.targetChanged.connect(this, "updateYawTarget()");
        

        ui.pitchEnabled.toggled.connect(auv.autopilots.PITCH, "setEnabled(boolean)");
        auv.autopilots.PITCH.enabledChanged.connect(ui.pitchEnabled, "setChecked(boolean)");
        ui.pitchTarget.valueChanged.connect(this, "updatePitchTarget(double)");
        auv.autopilots.PITCH.targetChanged.connect(this, "updatePitchTarget()");

        auv.depthChanged.connect(this, "updateDepthActual(float)");
        auv.orientationChanged.connect(this, "updateOrientationActual(floatYPR)");
    }

    public void updateDepthActual(float depth){
        ui.depthActual.setText("Actual: " + depth);
    }
    
    public void updateDepthTarget(){
        ui.depthValue.setValue(auv.autopilots.DEPTH.getTarget());
    }
    
    public void updateDepthTarget(double d){
        auv.autopilots.DEPTH.setTarget((float)d);
    }
    
    public void updateYawTarget(){
        ui.yawTarget.setValue(auv.autopilots.YAW.getTarget());
    }
    
    public void updateYawTarget(double d){
        auv.autopilots.YAW.setTarget((float)d);
    }
    
    public void updateOrientationActual(floatYPR orientation){
        ui.pitchActual.setText("Actual: " + orientation.pitch);
        ui.yawActual.setText("Actual: " + orientation.yaw);
    }
    
    public void updatePitchTarget(){
        ui.pitchTarget.setValue(auv.autopilots.PITCH.getTarget());
    }
    
    public void updatePitchTarget(double d){
        auv.autopilots.PITCH.setTarget((float)d);
    }
    
    public void setDebug(boolean state){
        MessageSocket.setDebug(state);
    }
    
    public void calibrateDepth(){
        auv.calibrateDepth((float)ui.foreOffset.value(), (float)ui.foreScale.value(),
                (float)ui.aftOffset.value(), (float)ui.aftScale.value());
    }
    
    @Override
    public void onDisconnect() {
       auv = null;
    }
    
	@Override
	public QGraphicsItemInterface getIconWidget() {
		return icon;
	}

	@Override
	public QWidget getScreenWidget() {
		return this;
	}

	@Override
	public String getScreenName() {
		return "Settings";
	}
}
