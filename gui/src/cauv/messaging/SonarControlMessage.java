package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class SonarControlMessage extends Message {
    int m_id = 32;
    protected int direction;
    protected int width;
    protected int gain;
    protected int range;
    protected int radialRes;
    protected int angularRes;

    private byte[] bytes;

    public void direction(int direction) {
        deserialise();
        this.direction = direction;
    }
    public int direction() {
        deserialise();
        return this.direction;
    }

    public void width(int width) {
        deserialise();
        this.width = width;
    }
    public int width() {
        deserialise();
        return this.width;
    }

    public void gain(int gain) {
        deserialise();
        this.gain = gain;
    }
    public int gain() {
        deserialise();
        return this.gain;
    }

    public void range(int range) {
        deserialise();
        this.range = range;
    }
    public int range() {
        deserialise();
        return this.range;
    }

    public void radialRes(int radialRes) {
        deserialise();
        this.radialRes = radialRes;
    }
    public int radialRes() {
        deserialise();
        return this.radialRes;
    }

    public void angularRes(int angularRes) {
        deserialise();
        this.angularRes = angularRes;
    }
    public int angularRes() {
        deserialise();
        return this.angularRes;
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

            s.writeShort(this.direction);
            s.writeShort(this.width);
            s.writeByte(this.gain);
            s.writeInt(this.range);
            s.writeInt(this.radialRes);
            s.writeByte(this.angularRes);

            return bs.toByteArray();
        }
    }

    public SonarControlMessage(){
        super(32, "sonarctl");
        this.bytes = null;
    }

    public SonarControlMessage(Integer direction, Integer width, Integer gain, Integer range, Integer radialRes, Integer angularRes) {
        super(32, "sonarctl");
        this.bytes = null;

        this.direction = direction;
        this.width = width;
        this.gain = gain;
        this.range = range;
        this.radialRes = radialRes;
        this.angularRes = angularRes;
    }

    public SonarControlMessage(byte[] bytes) {
        super(32, "sonarctl");
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
                    throw new IllegalArgumentException("Attempted to create SonarControlMessage with invalid id");
                }

                this.direction = s.readUnsignedShort();
                this.width = s.readUnsignedShort();
                this.gain = s.readUnsignedByte();
                this.range = s.readInt();
                this.radialRes = s.readInt();
                this.angularRes = s.readUnsignedByte();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
