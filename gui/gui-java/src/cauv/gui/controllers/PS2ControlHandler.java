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

    protected boolean enableL = false;
    protected boolean enableR = false;
    
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
                    motion.yaw(auv.autopilots.YAW.getTarget() + (yawRate));
                    motion.depth(auv.autopilots.DEPTH.getTarget() + (0.1f * depthRate));
                    motion.pitch(auv.autopilots.PITCH.getTarget() - (pitchRate));
                    
                    try {
                        Thread.sleep(100);
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
                if(enableL)
                    motion.strafe((int) ((value * value * value) * 127)); 
              break;
            case JOY_L_Y:
                if(enableL)
                    depthRate = (value * value * value);
                break;
            case JOY_R_X:
                if(enableR)
                    yawRate = (value * value * value);
                break;
            case JOY_R_Y:
                if(enableR)
                    pitchRate = (value * value * value);
                break;

            case JOY_L:
                if(value == 1){
                    enableL = !enableL;
                    System.out.println("L Enabled = "+enableL);
                }
                break;
            case JOY_R:
                if(value == 1){
                    enableR = !enableR;
                    System.out.println("R Enabled = "+enableR);
                }
                break;
            case X:
                if(value == 1)
                    motion.forward(127);
                else motion.forward(0);
                break;
            case R1:
                if(value == 1)
                    motion.forward((int) (auv.motors.PROP.get() + 16));
                break;
            case R2:
                if(value == 1)
                    motion.forward((int) (auv.motors.PROP.get() - 16));
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
                motion.yaw(auv.telemetry.ORIENTATION.getYaw());
                motion.depth(auv.telemetry.DEPTH.getDepth());
                motion.pitch(auv.telemetry.ORIENTATION.getPitch());
                break;
            case SQUARE:
                motion.stop();
                break;
            case CIRCLE:
                System.out.println("surfacing");
                motion.depth(0f);
                break;
        }
    }
}
