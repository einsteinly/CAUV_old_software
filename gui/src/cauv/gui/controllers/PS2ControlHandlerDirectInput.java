package cauv.gui.controllers;

import java.io.IOException;
import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MotionController;
import cauv.gamepad.PS2Controller;
import cauv.gamepad.PS2Controller.Button;
import cauv.gui.Main;

public class PS2ControlHandlerDirectInput implements cauv.gamepad.PS2InputHandler {

    AUV auv;
    MotionController motion;
    
    protected boolean ignoreInput = false;

    public void enable(){
        this.ignoreInput = false;
    }

    public void disable(){
        this.ignoreInput = true;
    }
    
    public PS2ControlHandlerDirectInput(AUV auv) {
        this.auv = auv;
        motion = new MotionController(auv);
        
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

        System.out.println(button + " = " + value);
        
        // crappy hack to limit the jitter on the controller 
        if(Math.abs(value) < 0.1) value = 0;
        
        if (ignoreInput) return;
        switch (button) {
            case JOY_L_X:
            	motion.strafe((int)((value) * 127));
                break;
            case JOY_L_Y:
            	motion.depth((int) ((value) * 127));
                break;
            case JOY_R_X:
            	motion.yaw((int) ((value) * 127));
                break;
            case JOY_R_Y:
            	motion.pitch((int) ((-value) * 127));
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
                break;
            case SQUARE:
                motion.stop();
                break;
            case CIRCLE:
                break;
        }
    }
}
