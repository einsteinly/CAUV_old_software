package cauv.gui.views;

import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MessageSocket;
import cauv.gui.ScreenView;
import cauv.messaging.MessageSource;

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
		ui.saveSettingsButton.clicked.connect(this, "save()");
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
        ui.sendDepth.released.connect(this, "calibrateDepth()");
    }
    
    public void setDebug(boolean state){
        MessageSocket.setDebug(state);
    }
    
    public void calibrateDepth(){
        auv.calibrateDepth((float)ui.depth.value());
    }
    
    @Override
    public void onDisconnect(AUV auv) {
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
