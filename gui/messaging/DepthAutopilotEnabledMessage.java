package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class DepthAutopilotEnabledMessage extends Message {
    int m_id = 61;
    public boolean enabled;
    public int target;

    public void enabled(boolean enabled){
        this.enabled = enabled;
    }
    public boolean enabled(){
        return this.enabled;
    }

    public void target(int target){
        this.target = target;
    }
    public int target(){
        return this.target;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeBoolean(this.enabled);
        s.writeShort(this.target);

        return bs.toByteArray();
    }

    public DepthAutopilotEnabledMessage(){
        super(61, "control");
    }

    public DepthAutopilotEnabledMessage(boolean enabled, int target) {

        this.enabled = enabled;
        this.target = target;
    }

    public DepthAutopilotEnabledMessage(byte[] bytes) throws IOException {
        super(61, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create DepthAutopilotEnabledMessage with invalid id");
        }

        this.enabled = s.readBoolean();
        this.target = s.readUnsignedShort();
    }
}
