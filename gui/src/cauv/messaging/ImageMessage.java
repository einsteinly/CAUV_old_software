package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ImageMessage extends Message {
    int m_id = 4;
    public CameraID source;
    public Image image;
    public TimeStamp time;

    public void source(CameraID source){
        this.source = source;
    }
    public CameraID source(){
        return this.source;
    }

    public void image(Image image){
        this.image = image;
    }
    public Image image(){
        return this.image;
    }

    public void time(TimeStamp time){
        this.time = time;
    }
    public TimeStamp time(){
        return this.time;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        this.source.writeInto(s);
        this.image.writeInto(s);
        this.time.writeInto(s);

        return bs.toByteArray();
    }

    public ImageMessage(){
        super(4, "image");
    }

    public ImageMessage(CameraID source, Image image, TimeStamp time) {
        super(4, "image");
        this.source = source;
        this.image = image;
        this.time = time;
    }

    public ImageMessage(byte[] bytes) throws IOException {
        super(4, "image");
        System.out.println("debug 1");
        
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);

        System.out.println("debug 2");
        LEDataInputStream s = new LEDataInputStream(bs);
        

        System.out.println("debug 3");
        int buf_id = s.readInt();

        System.out.println("debug 4");
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create ImageMessage with invalid id");
        }

        System.out.println("debug 5");
        this.source = CameraID.readFrom(s);

        System.out.println("debug 6");
        this.image = Image.readFrom(s);

        System.out.println("debug 7");
        this.time = TimeStamp.readFrom(s);
        

        System.out.println("debug 8");
    }
}
