package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class HoughCirclesMessage extends Message {
    int m_id = 131;
    protected Vector< Circle > circles;

    private byte[] bytes;

    public void circles(Vector< Circle > circles) {
        deserialise();
        this.circles = circles;
    }
    public Vector< Circle > circles() {
        deserialise();
        return this.circles;
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

            s.writeInt(this.circles.size());
            for (int circles_i = 0; circles_i < this.circles.size(); circles_i++)
            {
                Circle circles_val = this.circles.get(circles_i);
                circles_val.writeInto(s);
            }

            return bs.toByteArray();
        }
    }

    public HoughCirclesMessage(){
        super(131, "processing");
        this.bytes = null;
    }

    public HoughCirclesMessage(Vector< Circle > circles) {
        super(131, "processing");
        this.bytes = null;

        this.circles = circles;
    }

    public HoughCirclesMessage(byte[] bytes) {
        super(131, "processing");
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
                    throw new IllegalArgumentException("Attempted to create HoughCirclesMessage with invalid id");
                }

                this.circles = new Vector< Circle >();
                long circles_len = s.readInt();
                for (int circles_i = 0; circles_i < circles_len; circles_i++)
                {
                    Circle circles_val;
                    circles_val = Circle.readFrom(s);
                    this.circles.add(circles_i, circles_val);
                }

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
