package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ControllerStateMessage extends Message {
    int m_id = 100;
    protected Controller contoller;
    protected float mv;
    protected float error;
    protected float derror;
    protected float ierror;
    protected MotorDemand demand;

    private byte[] bytes;

    public void contoller(Controller contoller) {
        deserialise();
        this.contoller = contoller;
    }
    public Controller contoller() {
        deserialise();
        return this.contoller;
    }

    public void mv(float mv) {
        deserialise();
        this.mv = mv;
    }
    public float mv() {
        deserialise();
        return this.mv;
    }

    public void error(float error) {
        deserialise();
        this.error = error;
    }
    public float error() {
        deserialise();
        return this.error;
    }

    public void derror(float derror) {
        deserialise();
        this.derror = derror;
    }
    public float derror() {
        deserialise();
        return this.derror;
    }

    public void ierror(float ierror) {
        deserialise();
        this.ierror = ierror;
    }
    public float ierror() {
        deserialise();
        return this.ierror;
    }

    public void demand(MotorDemand demand) {
        deserialise();
        this.demand = demand;
    }
    public MotorDemand demand() {
        deserialise();
        return this.demand;
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

            this.contoller.writeInto(s);
            s.writeFloat(this.mv);
            s.writeFloat(this.error);
            s.writeFloat(this.derror);
            s.writeFloat(this.ierror);
            this.demand.writeInto(s);

            return bs.toByteArray();
        }
    }

    public ControllerStateMessage(){
        super(100, "gui");
        this.bytes = null;
    }

    public ControllerStateMessage(Controller contoller, Float mv, Float error, Float derror, Float ierror, MotorDemand demand) {
        super(100, "gui");
        this.bytes = null;

        this.contoller = contoller;
        this.mv = mv;
        this.error = error;
        this.derror = derror;
        this.ierror = ierror;
        this.demand = demand;
    }

    public ControllerStateMessage(byte[] bytes) {
        super(100, "gui");
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
                    throw new IllegalArgumentException("Attempted to create ControllerStateMessage with invalid id");
                }

                this.contoller = Controller.readFrom(s);
                this.mv = s.readFloat();
                this.error = s.readFloat();
                this.derror = s.readFloat();
                this.ierror = s.readFloat();
                this.demand = MotorDemand.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
