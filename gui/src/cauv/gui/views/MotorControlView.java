package cauv.gui.views;

import cauv.auv.AUV;
import cauv.gui.ScreenView;

import com.trolltech.qt.gui.*;

public class MotorControlView extends QWidget implements ScreenView {

    Ui_MotorControlView ui = new Ui_MotorControlView();
    
    AUV auv;
    
    public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();

    public MotorControlView() {
        ui.setupUi(this);        
    }

    public MotorControlView(QWidget parent) {
        super(parent);
        ui.setupUi(this);
    }

	@Override
	public QGraphicsItemInterface getIconWidget() {
		return icon;
	}

	@Override
	public String getScreenName() {
		return "Motor Controls";
	}

	@Override
	public QWidget getScreenWidget() {
		return this;
	}

	@Override
	public void onConnect(AUV auv) {
	    this.auv = auv;
	    
		auv.motors.PROP.speedChanged.connect(ui.sliderProp, "setValue(int)");
		auv.motors.HBOW.speedChanged.connect(ui.sliderHBow, "setValue(int)");
		auv.motors.HSTERN.speedChanged.connect(ui.sliderHStern, "setValue(int)");
		auv.motors.VBOW.speedChanged.connect(ui.sliderVBow, "setValue(int)");
		auv.motors.VSTERN.speedChanged.connect(ui.sliderVStern, "setValue(int)");
		
        ui.sliderProp.sliderMoved.connect(auv.motors.PROP, "set(Integer)");
        ui.sliderHBow.sliderMoved.connect(auv.motors.HBOW, "set(Integer)");
        ui.sliderHStern.sliderMoved.connect(auv.motors.HSTERN, "set(Integer)");
        ui.sliderVBow.sliderMoved.connect(auv.motors.VBOW, "set(Integer)");
        ui.sliderVStern.sliderMoved.connect(auv.motors.VSTERN, "set(Integer)");
        
        ui.shutdownButton.released.connect(auv, "stopAllMotors()");
	}
	
	public void setPropSpeed(){
	    auv.motors.PROP.set(ui.sliderProp.value());
	}
	
	@Override
	public void onDisconnect() {
	    // TODO Auto-generated method stub
	}
}
