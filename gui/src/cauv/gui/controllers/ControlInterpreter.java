package cauv.gui.controllers;

import cauv.auv.AUV;
import cauv.auv.MotionController;

public abstract class ControlInterpreter {

	protected AUV auv;
	protected MotionController motion;
	
	public ControlInterpreter(AUV auv) {
		this.auv = auv;
		this.motion = new MotionController(auv);
	}

	public abstract void panUp();
	public abstract void panDown();
	public abstract void panRight();
	public abstract void panLeft();
	public abstract void focus(float x, float y);
}
