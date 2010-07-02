package cauv.auv;

public class MotionController {
	
	protected AUV auv;
	
	public MotionController(AUV auv) {
		this.auv = auv;
	}
	
	public void yaw(int speed){
        auv.motors.HBOW.set(speed);
        auv.motors.HSTERN.set(-speed);
	}
	
	public void yaw(float yaw){
		auv.autopilots.YAW.setTarget(yaw);
	}
	
	public void pitch(int speed){
        auv.motors.VBOW.set(speed);
        auv.motors.VSTERN.set(-speed);
	}
	
	public void pitch(float pitch){
		auv.autopilots.PITCH.setTarget(pitch);
	}

	public void strafe(int speed){
        auv.motors.HBOW.set(speed);
        auv.motors.HSTERN.set(speed);
	}
	
	public void depth(int speed){
        auv.motors.VBOW.set(speed);
        auv.motors.VSTERN.set(speed);
	}
	
	public void depth(float depth){
		auv.autopilots.DEPTH.setTarget(depth);
	}
	
	public void forward(int speed){
        auv.motors.PROP.set(speed);
	}
	
    public void stop() {
        auv.motors.HBOW.set(0);
        auv.motors.HSTERN.set(0);
        auv.motors.PROP.set(0);
        auv.motors.VBOW.set(0);
        auv.motors.VSTERN.set(0);
    }
}
