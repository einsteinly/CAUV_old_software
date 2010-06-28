package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

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
        DataOutputStream s = new DataOutputStream(bs);
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

        this.source = source;
        this.image = image;
        this.time = time;
    }

    public ImageMessage(byte[] bytes) throws IOException {
        super(4, "image");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create ImageMessage with invalid id");
        }

        this.source.readFrom(s);
        this.image.readFrom(s);
        this.time.readFrom(s);
    }
}
