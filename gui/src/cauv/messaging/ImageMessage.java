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
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create ImageMessage with invalid id");
        }

        this.source = CameraID.readFrom(s);
        this.image = Image.readFrom(s);
        this.time = TimeStamp.readFrom(s);
    }
}
