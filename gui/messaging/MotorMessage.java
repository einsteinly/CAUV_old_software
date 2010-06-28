package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class MotorMessage extends Message {
    int m_id = 2;
    public MotorID motorId;
    public byte speed;

    public void motorId(MotorID motorId){
        this.motorId = motorId;
    }
    public MotorID motorId(){
        return this.motorId;
    }

    public void speed(byte speed){
        this.speed = speed;
    }
    public byte speed(){
        return this.speed;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        this.motorId.writeInto(s);
        s.writeByte(this.speed);

        return bs.toByteArray();
    }

    public MotorMessage(){
        super(2, "control");
    }

    public MotorMessage(MotorID motorId, byte speed) {

        this.motorId = motorId;
        this.speed = speed;
    }

    public MotorMessage(byte[] bytes) throws IOException {
        super(2, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create MotorMessage with invalid id");
        }

        this.motorId.readFrom(s);
        this.speed = s.readByte();
    }
}
