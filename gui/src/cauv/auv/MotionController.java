package cauv.auv;

public class MotionController {
	
	protected AUV auv;
	
	public MotionController(AUV auv) {
		this.auv = auv;
	}
	
	public void yaw(int speed){
        auv.motors.VBOW.setSpeed(speed);
        auv.motors.VSTERN.setSpeed(-speed);
	}
	
	public void yaw(float yaw){
		auv.autopilots.YAW.setTarget(yaw);
	}
	
	public void pitch(int speed){
        auv.motors.HBOW.setSpeed(speed);
        auv.motors.HSTERN.setSpeed(-speed);
	}
	
	public void pitch(float pitch){
		auv.autopilots.PITCH.setTarget(pitch);
	}

	public void strafe(int speed){
        auv.motors.HBOW.setSpeed(speed);
        auv.motors.HSTERN.setSpeed(speed);
	}
	
	public void depth(int speed){
        auv.motors.HBOW.setSpeed(speed);
        auv.motors.HSTERN.setSpeed(speed);
	}
	
	public void depth(float depth){
		auv.autopilots.DEPTH.setTarget(depth);
	}
	
	public void forward(int speed){
        auv.motors.PROP.setSpeed(speed);
	}
	
    public void stop() {
        auv.motors.HBOW.setSpeed(0);
        auv.motors.HSTERN.setSpeed(0);
        auv.motors.PROP.setSpeed(0);
        auv.motors.VBOW.setSpeed(0);
        auv.motors.VSTERN.setSpeed(0);
    }
}
