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
    MotionController motion;
    
    protected boolean ignoreInput = true;

    public PS2ControlHandler(AUV auv) {
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
    public void onPressed(final Button button, final float value) {

        if (ignoreInput) return;
        switch (button) {
            case JOY_L_X:
            	motion.strafe((int)(value * value * value) * 128);
                break;
            case JOY_L_Y:
            	motion.depth((int) ((value * value * value) * 128));
                break;
            case JOY_R_X:
            	motion.yaw((int) ((value * value * value) * 128));
                break;
            case JOY_R_Y:
            	motion.pitch((int) ((value * value * value) * 128));
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
