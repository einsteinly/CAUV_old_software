package cauv.gui.controllers;

import java.io.IOException;
import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MotionController;
import cauv.gamepad.PS2Controller;
import cauv.gamepad.PS2Controller.Button;
import cauv.gui.Main;

public class PS2ControlHandler implements cauv.gamepad.PS2InputHandler {

    AUV auv;
    final MotionController motion;
    
    protected boolean ignoreInput = false;

    protected volatile float yawRate = 0;
    protected volatile float depthRate = 0;
    protected volatile float pitchRate = 0;
    
    public void enable(){
        this.ignoreInput = false;
    }

    public void disable(){
        this.ignoreInput = true;
    }
    
    public PS2ControlHandler(final AUV auv) {
        this.auv = auv;
        motion = new MotionController(auv);
        
        final Thread t = new Thread(){
            public void run(){
                while(true){
                    motion.yaw(auv.autopilots.YAW.getTarget() + (0.1f * yawRate));
                    motion.depth(auv.autopilots.DEPTH.getTarget() + (0.1f * depthRate));
                    motion.pitch(auv.autopilots.PITCH.getTarget() - (0.1f * pitchRate));
                    
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                    }
                }
            }
        };
        t.setDaemon(true);
        t.start();
        
        try {
            Main.trace(PS2Controller.listControllers());
            PS2Controller ps2 = new PS2Controller(Config.GAMEPAD_ID);
            ps2.registerObserver(this);
        } catch (IOException e) {
            System.err.println("Gamepad not availible: " + e.getMessage());
        }
    }

    @Override
    public void onPressed(final Button button, float value) {

        //Main.trace(button + " = " + value);
        
        // crappy hack to limit the jitter on the controller 
        if(Math.abs(value) < 0.2) value = 0;
        
        if (ignoreInput) return;
        switch (button) {
            case JOY_L_X: 
                motion.strafe((int) (value * 127)); 
              break;
            case JOY_L_Y:
                depthRate = -value;
                break;
            case JOY_R_X:
                yawRate = value;
                break;
            case JOY_R_Y:
                pitchRate = value;
                break;

            case JOY_L:
                break;
            case JOY_R:
                break;
            case X:
                break;
            case R1:
            	motion.forward((int) (auv.motors.PROP.getSpeed() + 16));
                break;
            case R2:
            	motion.forward((int) (auv.motors.PROP.getSpeed() - 16));
                break;

            case UP:
                break;
            case DOWN:
                break;
            case LEFT:
                break;
            case RIGHT:
                break;

            case TRIANGLE:
                motion.yaw(auv.getOrientation().yaw);
                motion.depth(auv.getDepth());
                motion.pitch(auv.getOrientation().pitch);
                break;
            case SQUARE:
                motion.stop();
                break;
            case CIRCLE:
                break;
        }
    }
}
