package cauv.gui.views;

import cauv.auv.AUV;
import cauv.gui.ScreenView;
import cauv.gui.components.ControlableVideoScreen;
import cauv.gui.controllers.ControlInterpreter;
import cauv.gui.views.Ui_CameraFeeds;

import com.trolltech.qt.core.QTimer;
import com.trolltech.qt.gui.*;

public class CameraFeeds extends QWidget implements ScreenView {

	public static transient final float DEPTH_STEP = 0.2f;
	public static transient final int STRAFE_SPEED = 100;

	public static transient final float H_FOV = 120;
	public static transient final float V_FOV = 120;

	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
	Ui_CameraFeeds ui = new Ui_CameraFeeds();

	protected AUV auv = null;

	ControlInterpreter forwardCamInterpreter;
	ControlInterpreter downwardCamInterpreter;
	ControlInterpreter sonarInterpreter;

	public CameraFeeds() {
	    QTimer timer = new QTimer(this);
	    timer.setInterval(200);
	    timer.setSingleShot(false);
	    timer.timeout.connect(this, "updateValues()");
	    timer.start();
	    
		ui.setupUi(this);
		//forward cam setup
		ui.forwardCam.setXAxisScale(V_FOV);
		ui.forwardCam.setYAxisScale(H_FOV);
		ui.forwardCamIcon.clicked.connect(this, "showForwardCam(QMouseEvent)");
		//downward cam setup
		ui.downwardCam.setXAxisScale(V_FOV);
		ui.downwardCam.setYAxisScale(H_FOV);
		ui.downwardCamIcon.clicked.connect(this, "showDownwardCam(QMouseEvent)");
		ui.downwardCam.setVisibleElements(ControlableVideoScreen.SHOW_TEXT);
		//sonar setup
		ui.sonarCam.setVisibleElements(ControlableVideoScreen.SHOW_TEXT);
		ui.sonarIcon.clicked.connect(this, "showSonar(QMouseEvent)");

        ui.direction.valueChanged.connect(this, "updateSonarParams()");
        ui.width.valueChanged.connect(this, "updateSonarParams()");
        ui.gain.valueChanged.connect(this, "updateSonarParams()");
        ui.range.valueChanged.connect(this, "updateSonarParams()");
        ui.radialRes.valueChanged.connect(this, "updateSonarParams()");
        ui.angularRes.valueChanged.connect(this, "updateSonarParams()");
		
	}

	public void updateSonarParams(int direction, int width, int gain, int range, int radialRes, int angularRes){
	    ui.direction.setValue(direction);
	    ui.width.setValue(width);
	    ui.gain.setValue(gain);
	    ui.range.setValue(range);
        ui.radialRes.setValue(radialRes);
        ui.angularRes.setValue(angularRes);
	}
	
	public void updateSonarParams(){
	    if(auv == null)
	        return;
	    auv.cameras.SONAR.setParams(ui.direction.value(), 
	            ui.width.value(), ui.gain.value(),
	            ui.range.value(), ui.radialRes.value(),
	            ui.angularRes.value());
	}
	
	public void onConnect(AUV auv) {

		this.auv = auv;

		auv.autopilots.DEPTH.targetChanged.connect(this, "updateValues()");
		auv.autopilots.YAW.targetChanged.connect(this, "updateValues()");
		auv.autopilots.PITCH.targetChanged.connect(this, "updateValues()");
        
		auv.cameras.SONAR.paramsChanged.connect(this, "updateSonarParams(int, int, int, int, int, int)");
		
		
		// the control interpreters map the buttons on the camera screens to 
		// sensible actions for that view
		forwardCamInterpreter = new ControlInterpreter(auv) {
			public void panUp() {
				this.motion.depth(auv.autopilots.DEPTH.getTarget() + DEPTH_STEP);
			}

			public void panRight() {
				this.motion.strafe(STRAFE_SPEED);
			}

			public void panLeft() {
				this.motion.strafe(-STRAFE_SPEED);
			}

			public void panDown() {
				this.motion.depth(auv.autopilots.DEPTH.getTarget() - DEPTH_STEP);
			}

			public void focus(float x, float y) {
				this.motion.yaw(x);
				this.motion.pitch(y);
			}
		};
		
		this.ui.forwardCam.upClick.connect(forwardCamInterpreter,"panUp()");
		this.ui.forwardCam.downClick.connect(forwardCamInterpreter,"panDown()");
		this.ui.forwardCam.leftClick.connect(forwardCamInterpreter,"panLeft()");
		this.ui.forwardCam.rightClick.connect(forwardCamInterpreter, "panRight()");
		this.ui.forwardCam.directionChange.connect(forwardCamInterpreter, "focus(float, float)");

		
		downwardCamInterpreter = new ControlInterpreter(auv) {
			public void panUp() {
				this.motion.forward(STRAFE_SPEED);
			}

			public void panRight() {
				this.motion.strafe(STRAFE_SPEED);
			}

			public void panLeft() {
				this.motion.strafe(-STRAFE_SPEED);
			}

			public void panDown() {
				this.motion.forward(-STRAFE_SPEED);
			}

			public void focus(float x, float y) {
				this.motion.yaw(x);
				this.motion.pitch(y);
			}
		};

		this.ui.downwardCam.upClick.connect(downwardCamInterpreter,"panUp()");
		this.ui.downwardCam.downClick.connect(downwardCamInterpreter,"panDown()");
		this.ui.downwardCam.leftClick.connect(downwardCamInterpreter,"panLeft()");
		this.ui.downwardCam.rightClick.connect(downwardCamInterpreter, "panRight()");
		this.ui.downwardCam.directionChange.connect(downwardCamInterpreter, "focus(float, float)");

		// sonar uses the same controls as the downward cam
		this.ui.sonarCam.upClick.connect(downwardCamInterpreter,"panUp()");
		this.ui.sonarCam.downClick.connect(downwardCamInterpreter,"panDown()");
		this.ui.sonarCam.leftClick.connect(downwardCamInterpreter,"panLeft()");
		this.ui.sonarCam.rightClick.connect(downwardCamInterpreter, "panRight()");
		this.ui.sonarCam.directionChange.connect(downwardCamInterpreter, "focus(float, float)");

	}

	@Override
	public void onDisconnect(AUV auv) {
	    this.auv = null;
	}
	
	public void updateValues() {	    
	    String s;
	    if(auv == null) {
	        s = "Auv not connected";
	    } else {
    		s = "Depth [Target]: " + auv.autopilots.DEPTH.getTarget() + "\n";
    		s += "Depth [Actual]: " + auv.getDepth() + "\n\n";
    		
    		s += "Yaw [Target]: " + auv.autopilots.YAW.getTarget() + "\n";
    		s += "Yaw [Actual]: " + auv.getOrientation().yaw + "\n\n";
    		
    		s += "Pitch [Target]: " + auv.autopilots.PITCH.getTarget() + "\n";
    		s += "Pitch [Actual]: " + auv.getOrientation().pitch;

    		this.ui.forwardCam.setDotLocation(auv.autopilots.YAW.getTarget(), auv.autopilots.PITCH.getTarget());
            this.ui.downwardCam.setDotLocation(auv.autopilots.YAW.getTarget(), auv.autopilots.PITCH.getTarget());
            this.ui.sonarCam.setDotLocation(auv.autopilots.YAW.getTarget(), auv.autopilots.PITCH.getTarget());
	    }
	    
		this.ui.forwardCam.setText(s);
		this.ui.downwardCam.setText(s);
		this.ui.sonarCam.setText(s);
	}

	public void showSonar(QMouseEvent event){
		this.ui.feeds.setCurrentWidget(ui.sonarCamPage);
	}
	
	public void showForwardCam(QMouseEvent event){
		this.ui.feeds.setCurrentWidget(ui.forwardCamPage);
	}
	
	public void showDownwardCam(QMouseEvent event){
		this.ui.feeds.setCurrentWidget(ui.downwardCamPage);
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
		return "Camera Feeds";
	}
}
