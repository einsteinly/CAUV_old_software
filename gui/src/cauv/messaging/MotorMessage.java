package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class MotorMessage extends Message {
    int m_id = 2;
    protected MotorID motorId;
    protected byte speed;

    private byte[] bytes;

    public void motorId(MotorID motorId) {
        deserialise();
        this.motorId = motorId;
    }
    public MotorID motorId() {
        deserialise();
        return this.motorId;
    }

    public void speed(byte speed) {
        deserialise();
        this.speed = speed;
    }
    public byte speed() {
        deserialise();
        return this.speed;
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

            this.motorId.writeInto(s);
            s.writeByte(this.speed);

            return bs.toByteArray();
        }
    }

    public MotorMessage(){
        super(2, "control");
        this.bytes = null;
    }

    public MotorMessage(MotorID motorId, Byte speed) {
        super(2, "control");
        this.bytes = null;

        this.motorId = motorId;
        this.speed = speed;
    }

    public MotorMessage(byte[] bytes) {
        super(2, "control");
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
                    throw new IllegalArgumentException("Attempted to create MotorMessage with invalid id");
                }

                this.motorId = MotorID.readFrom(s);
                this.speed = s.readByte();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
