package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class SonarControlMessage extends Message {
    int m_id = 32;
    public int direction;
    public int width;
    public int gain;
    public int range;
    public int radialRes;
    public int angularRes;

    public void direction(int direction){
        this.direction = direction;
    }
    public int direction(){
        return this.direction;
    }

    public void width(int width){
        this.width = width;
    }
    public int width(){
        return this.width;
    }

    public void gain(int gain){
        this.gain = gain;
    }
    public int gain(){
        return this.gain;
    }

    public void range(int range){
        this.range = range;
    }
    public int range(){
        return this.range;
    }

    public void radialRes(int radialRes){
        this.radialRes = radialRes;
    }
    public int radialRes(){
        return this.radialRes;
    }

    public void angularRes(int angularRes){
        this.angularRes = angularRes;
    }
    public int angularRes(){
        return this.angularRes;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeShort(this.direction);
        s.writeShort(this.width);
        s.writeByte(this.gain);
        s.writeInt(this.range);
        s.writeInt(this.radialRes);
        s.writeByte(this.angularRes);

        return bs.toByteArray();
    }

    public SonarControlMessage(){
        super(32, "sonarctl");
    }

    public SonarControlMessage(int direction, int width, int gain, int range, int radialRes, int angularRes) {

        this.direction = direction;
        this.width = width;
        this.gain = gain;
        this.range = range;
        this.radialRes = radialRes;
        this.angularRes = angularRes;
    }

    public SonarControlMessage(byte[] bytes) throws IOException {
        super(32, "sonarctl");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
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
    }
}
