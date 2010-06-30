package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class MotorStateMessage extends Message {
    int m_id = 101;
    public MotorDemand demand;

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

        this.demand.writeInto(s);

        return bs.toByteArray();
    }

    public MotorStateMessage(){
        super(101, "gui");
    }

    public MotorStateMessage(MotorDemand demand) {
        super(101, "gui");
        this.demand = demand;
    }

    public MotorStateMessage(byte[] bytes) throws IOException {
        super(101, "gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create MotorStateMessage with invalid id");
        }

        this.demand = MotorDemand.readFrom(s);
    }
}
