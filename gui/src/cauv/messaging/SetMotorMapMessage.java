package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class SetMotorMapMessage extends Message {
    int m_id = 84;
    protected MotorID motor;
    protected MotorMap mapping;

    private byte[] bytes;

    public void motor(MotorID motor) {
        deserialise();
        this.motor = motor;
    }
    public MotorID motor() {
        deserialise();
        return this.motor;
    }

    public void mapping(MotorMap mapping) {
        deserialise();
        this.mapping = mapping;
    }
    public MotorMap mapping() {
        deserialise();
        return this.mapping;
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

            this.motor.writeInto(s);
            this.mapping.writeInto(s);

            return bs.toByteArray();
        }
    }

    public SetMotorMapMessage(){
        super(84, "control");
        this.bytes = null;
    }

    public SetMotorMapMessage(MotorID motor, MotorMap mapping) {
        super(84, "control");
        this.bytes = null;

        this.motor = motor;
        this.mapping = mapping;
    }

    public SetMotorMapMessage(byte[] bytes) {
        super(84, "control");
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
                    throw new IllegalArgumentException("Attempted to create SetMotorMapMessage with invalid id");
                }

                this.motor = MotorID.readFrom(s);
                this.mapping = MotorMap.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
