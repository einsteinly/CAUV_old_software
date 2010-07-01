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
    protected CameraID source;
    protected Image image;
    protected TimeStamp time;

    private byte[] bytes;

    public void source(CameraID source) {
        deserialise();
        this.source = source;
    }
    public CameraID source() {
        deserialise();
        return this.source;
    }

    public void image(Image image) {
        deserialise();
        this.image = image;
    }
    public Image image() {
        deserialise();
        return this.image;
    }

    public void time(TimeStamp time) {
        deserialise();
        this.time = time;
    }
    public TimeStamp time() {
        deserialise();
        return this.time;
    }


    public byte[] toBytes() throws IOException {
        if (bytes != null)
        {
            return bytes;
        }
        else
        {
            ByteArrayOutputStream bs = new ByteArrayOutputStream();
            LEDataOutputStream s = new LEDataOutputStream(bs);
            s.writeInt(m_id);

            this.source.writeInto(s);
            this.image.writeInto(s);
            this.time.writeInto(s);

            return bs.toByteArray();
        }
    }

    public ImageMessage(){
        super(4, "image");
        this.bytes = null;
    }

    public ImageMessage(CameraID source, Image image, TimeStamp time) {
        super(4, "image");
        this.bytes = null;

        this.source = source;
        this.image = image;
        this.time = time;
    }

    public ImageMessage(byte[] bytes) {
        super(4, "image");
        this.bytes = bytes;
    }

    public void deserialise() {
        try { 
            if (bytes != null)
            {
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

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
