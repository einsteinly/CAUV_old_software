package cauv.gui.views;

import cauv.auv.AUV;
import cauv.gui.ScreenView;
import cauv.gui.controllers.ControlInterpreter;
import cauv.gui.views.Ui_CameraFeeds;

import com.trolltech.qt.gui.*;

public class CameraFeeds extends QWidget implements ScreenView {

	public static transient final float DEPTH_STEP = 0.2f;
	public static transient final int STRAFE_SPEED = 100;

	public static transient final float H_FOV = 120;
	public static transient final float V_FOV = 120;

	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
	Ui_CameraFeeds ui = new Ui_CameraFeeds();

	protected AUV auv;

	ControlInterpreter forwardCamInterpreter;
	ControlInterpreter downwardCamInterpreter;
	ControlInterpreter sonarInterpreter;

	public CameraFeeds() {
		ui.setupUi(this);
	}

	public void onConnect(AUV auv) {

		this.auv = auv;

		auv.autopilots.DEPTH.targetChanged.connect(this, "updateText()");
		auv.autopilots.YAW.targetChanged.connect(this, "updateText()");
		auv.autopilots.PITCH.targetChanged.connect(this, "updateText()");

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
				this.motion
						.depth(auv.autopilots.DEPTH.getTarget() - DEPTH_STEP);
			}

			public void focus(float x, float y) {
				this.motion.yaw(x);
				this.motion.pitch(y);
			}
		};

		this.ui.controlableVideoScreen.upClick.connect(forwardCamInterpreter,"panUp()");
		this.ui.controlableVideoScreen.downClick.connect(forwardCamInterpreter,"panDown()");
		this.ui.controlableVideoScreen.leftClick.connect(forwardCamInterpreter,"panLeft()");
		this.ui.controlableVideoScreen.rightClick.connect(forwardCamInterpreter, "panRight()");
		this.ui.controlableVideoScreen.directionChange.connect(forwardCamInterpreter, "focus(float, float)");

	}

	public void updateText() {
		String s = "Depth [Target]: " + auv.autopilots.DEPTH.getTarget() + "\n";
		s += "Depth [Actual]: " + auv.getDepth() + "\n\n";
		
		s += "Yaw [Target]: " + auv.autopilots.YAW.getTarget() + "\n";
		s += "Yaw [Actual]: " + auv.getOrientation().yaw + "\n\n";
		
		s += "Pitch [Target]: " + auv.autopilots.PITCH.getTarget() + "\n";
		s += "Pitch [Actual]: " + auv.getOrientation().pitch;

		this.ui.controlableVideoScreen.setText(s);
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
