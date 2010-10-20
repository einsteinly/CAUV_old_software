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
    protected MotorDemand demand;

    private byte[] bytes;

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

            this.demand.writeInto(s);

            return bs.toByteArray();
        }
    }

    public MotorStateMessage(){
        super(101, "gui");
        this.bytes = null;
    }

    public MotorStateMessage(MotorDemand demand) {
        super(101, "gui");
        this.bytes = null;

        this.demand = demand;
    }

    public MotorStateMessage(byte[] bytes) {
        super(101, "gui");
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
                    throw new IllegalArgumentException("Attempted to create MotorStateMessage with invalid id");
                }

                this.demand = MotorDemand.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
