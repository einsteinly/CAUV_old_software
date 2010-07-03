package cauv.gamepad;

import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

/**
 *
 * @author Andy Pritchard
 */
public class PS2Reader extends Thread
{

    Controller controller = null;
    PS2Controller parent = null;

    public PS2Reader(Controller controller, PS2Controller parent)
    {
        this.parent = parent;
        this.controller = controller;
        this.setDaemon(true);
        this.start();
    }

    @Override
    public void run()
    {
        if (controller != null)
        while (true) {
            try {
                controller.poll();
                EventQueue queue = controller.getEventQueue();
                Event event = new Event();

                while (queue.getNextEvent(event)) {
                    parent.onEvent(event);
                }

                // Give the CPU a rest...
                Thread.sleep(50);
            }
            catch (InterruptedException ex) {
            }
        }
    }
}
