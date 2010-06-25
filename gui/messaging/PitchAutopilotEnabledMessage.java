package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class PitchAutopilotEnabledMessage extends Message {
    int m_id = 62;
    public boolean enabled;
    public short target;

    public void enabled(boolean enabled){
        this.enabled = enabled;
    }
    public boolean enabled(){
        return this.enabled;
    }

    public void target(short target){
        this.target = target;
    }
    public short target(){
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

    public PitchAutopilotEnabledMessage(){
        super(62, "control");
    }

    public PitchAutopilotEnabledMessage(boolean enabled, short target) {

        this.enabled = enabled;
        this.target = target;
    }

    public PitchAutopilotEnabledMessage(byte[] bytes) throws IOException {
        super(62, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create PitchAutopilotEnabledMessage with invalid id");
        }

        this.enabled = s.readBoolean();
        this.target = s.readShort();
    }
}
