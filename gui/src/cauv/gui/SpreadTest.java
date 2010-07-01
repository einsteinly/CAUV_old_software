package cauv.gui;

import java.io.IOException;
import java.net.UnknownHostException;

import cauv.auv.MessageSocket;
import cauv.messaging.ImageMessage;
import cauv.messaging.MessageObserver;
import cauv.messaging.TelemetryMessage;

public class SpreadTest extends MessageObserver {

    /**
     * @param args
     * @throws IOException 
     * @throws UnknownHostException 
     */
    public static void main(String[] args) throws UnknownHostException, IOException {
       MessageSocket s = new MessageSocket("leszek-laptop.local", 16708, "SpreadTest");
       s.joinGroup("control");
       s.joinGroup("debug");
       s.joinGroup("image");
       s.joinGroup("telemetry");
       s.joinGroup("sonarout");
       s.joinGroup("pressure");
       
       s.addObserver(new SpreadTest());
       
       
       System.out.println("connected");
       
       while(true) {
           Thread.yield();
       }
       
    }

}
