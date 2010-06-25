package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class BearingAutopilotEnabledMessage extends Message {
    int m_id = 60;
    public boolean enabled;
    public float target;

    public void enabled(boolean enabled){
        this.enabled = enabled;
    }
    public boolean enabled(){
        return this.enabled;
    }

    public void target(int target){
        this.target = target;
    }
    public float target(){
        return this.target;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeBoolean(this.enabled);
        s.writeFloat(this.target);

        return bs.toByteArray();
    }

    public BearingAutopilotEnabledMessage(){
        super(60, "control");
    }

    public BearingAutopilotEnabledMessage(boolean enabled, float target) {
        super(60, "control");
        this.enabled = enabled;
        this.target = target;
    }

    public BearingAutopilotEnabledMessage(byte[] bytes) throws IOException {
        super(60, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create BearingAutopilotEnabledMessage with invalid id");
        }

        this.enabled = s.readBoolean();
        this.target = s.readUnsignedShort();
    }
}
