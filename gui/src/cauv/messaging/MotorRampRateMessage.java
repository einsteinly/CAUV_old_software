package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class MotorRampRateMessage extends Message {
    int m_id = 83;
    protected int maxDelta;
    protected int updatesPerSecond;

    private byte[] bytes;

    public void maxDelta(int maxDelta) {
        deserialise();
        this.maxDelta = maxDelta;
    }
    public int maxDelta() {
        deserialise();
        return this.maxDelta;
    }

    public void updatesPerSecond(int updatesPerSecond) {
        deserialise();
        this.updatesPerSecond = updatesPerSecond;
    }
    public int updatesPerSecond() {
        deserialise();
        return this.updatesPerSecond;
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

            s.writeInt(this.maxDelta);
            s.writeInt(this.updatesPerSecond);

            return bs.toByteArray();
        }
    }

    public MotorRampRateMessage(){
        super(83, "control");
        this.bytes = null;
    }

    public MotorRampRateMessage(Integer maxDelta, Integer updatesPerSecond) {
        super(83, "control");
        this.bytes = null;

        this.maxDelta = maxDelta;
        this.updatesPerSecond = updatesPerSecond;
    }

    public MotorRampRateMessage(byte[] bytes) {
        super(83, "control");
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
                    throw new IllegalArgumentException("Attempted to create MotorRampRateMessage with invalid id");
                }

                this.maxDelta = s.readInt();
                this.updatesPerSecond = s.readInt();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
