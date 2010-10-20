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

	public abstract void panUp(boolean state);
	public abstract void panDown(boolean state);
	public abstract void panRight(boolean state);
	public abstract void panLeft(boolean state);
	public abstract void focus(float x, float y);
}
