package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class DepthAutopilotEnabledMessage extends Message {
    int m_id = 61;
    protected boolean enabled;
    protected float target;

    private byte[] bytes;

    public void enabled(boolean enabled) {
        deserialise();
        this.enabled = enabled;
    }
    public boolean enabled() {
        deserialise();
        return this.enabled;
    }

    public void target(float target) {
        deserialise();
        this.target = target;
    }
    public float target() {
        deserialise();
        return this.target;
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

            s.writeBoolean(this.enabled);
            s.writeFloat(this.target);

            return bs.toByteArray();
        }
    }

    public DepthAutopilotEnabledMessage(){
        super(61, "control");
        this.bytes = null;
    }

    public DepthAutopilotEnabledMessage(Boolean enabled, Float target) {
        super(61, "control");
        this.bytes = null;

        this.enabled = enabled;
        this.target = target;
    }

    public DepthAutopilotEnabledMessage(byte[] bytes) {
        super(61, "control");
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
                    throw new IllegalArgumentException("Attempted to create DepthAutopilotEnabledMessage with invalid id");
                }

                this.enabled = s.readBoolean();
                this.target = s.readFloat();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
