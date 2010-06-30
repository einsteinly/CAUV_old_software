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
    public Controller contoller;
    public float mv;
    public float error;
    public float derror;
    public float ierror;
    public MotorDemand demand;

    public void contoller(Controller contoller){
        this.contoller = contoller;
    }
    public Controller contoller(){
        return this.contoller;
    }

    public void mv(float mv){
        this.mv = mv;
    }
    public float mv(){
        return this.mv;
    }

    public void error(float error){
        this.error = error;
    }
    public float error(){
        return this.error;
    }

    public void derror(float derror){
        this.derror = derror;
    }
    public float derror(){
        return this.derror;
    }

    public void ierror(float ierror){
        this.ierror = ierror;
    }
    public float ierror(){
        return this.ierror;
    }

    public void demand(MotorDemand demand){
        this.demand = demand;
    }
    public MotorDemand demand(){
        return this.demand;
    }


    public byte[] toBytes() throws IOException {
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

    public ControllerStateMessage(){
        super(100, "gui");
    }

    public ControllerStateMessage(Controller contoller, Float mv, Float error, Float derror, Float ierror, MotorDemand demand) {
        super(100, "gui");
        this.contoller = contoller;
        this.mv = mv;
        this.error = error;
        this.derror = derror;
        this.ierror = ierror;
        this.demand = demand;
    }

    public ControllerStateMessage(byte[] bytes) throws IOException {
        super(100, "gui");
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
    }
}
