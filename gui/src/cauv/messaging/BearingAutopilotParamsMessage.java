package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class BearingAutopilotParamsMessage extends Message {
    int m_id = 70;
    protected float Kp;
    protected float Ki;
    protected float Kd;
    protected float scale;

    private byte[] bytes;

    public void Kp(float Kp) {
        deserialise();
        this.Kp = Kp;
    }
    public float Kp() {
        deserialise();
        return this.Kp;
    }

    public void Ki(float Ki) {
        deserialise();
        this.Ki = Ki;
    }
    public float Ki() {
        deserialise();
        return this.Ki;
    }

    public void Kd(float Kd) {
        deserialise();
        this.Kd = Kd;
    }
    public float Kd() {
        deserialise();
        return this.Kd;
    }

    public void scale(float scale) {
        deserialise();
        this.scale = scale;
    }
    public float scale() {
        deserialise();
        return this.scale;
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

            s.writeFloat(this.Kp);
            s.writeFloat(this.Ki);
            s.writeFloat(this.Kd);
            s.writeFloat(this.scale);

            return bs.toByteArray();
        }
    }

    public BearingAutopilotParamsMessage(){
        super(70, "control");
        this.bytes = null;
    }

    public BearingAutopilotParamsMessage(Float Kp, Float Ki, Float Kd, Float scale) {
        super(70, "control");
        this.bytes = null;

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.scale = scale;
    }

    public BearingAutopilotParamsMessage(byte[] bytes) {
        super(70, "control");
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
                    throw new IllegalArgumentException("Attempted to create BearingAutopilotParamsMessage with invalid id");
                }

                this.Kp = s.readFloat();
                this.Ki = s.readFloat();
                this.Kd = s.readFloat();
                this.scale = s.readFloat();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
