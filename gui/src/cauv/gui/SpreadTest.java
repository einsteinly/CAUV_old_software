package cauv.gui;

import java.io.IOException;
import java.net.UnknownHostException;

import cauv.auv.MessageSocket;
import cauv.messaging.ImageMessage;
import cauv.messaging.MessageObserver;

public class SpreadTest extends MessageObserver {

    /**
     * @param args
     * @throws IOException 
     * @throws UnknownHostException 
     */
    public static void main(String[] args) throws UnknownHostException, IOException {
       MessageSocket s = new MessageSocket("red-herring.local", 16707, "SpreadTest");
       s.joinGroup("control");
       s.joinGroup("debug");
       s.joinGroup("image");
       s.joinGroup("telemetry");
       s.joinGroup("sonarout");
       
       s.addObserver(new SpreadTest());
       
       s.joinGroup("pressure");
       
       System.out.println("connected");
       
       while(true) {
           Thread.yield();
       }
       
    }
    
    @Override
    public void onImageMessage(ImageMessage m) {
        System.out.println("image message: "+ m);
    }

}
