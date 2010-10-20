package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class DepthCalibrationMessage extends Message {
    int m_id = 80;
    protected float foreOffset;
    protected float foreMultiplier;
    protected float aftOffset;
    protected float aftMultiplier;

    private byte[] bytes;

    public void foreOffset(float foreOffset) {
        deserialise();
        this.foreOffset = foreOffset;
    }
    public float foreOffset() {
        deserialise();
        return this.foreOffset;
    }

    public void foreMultiplier(float foreMultiplier) {
        deserialise();
        this.foreMultiplier = foreMultiplier;
    }
    public float foreMultiplier() {
        deserialise();
        return this.foreMultiplier;
    }

    public void aftOffset(float aftOffset) {
        deserialise();
        this.aftOffset = aftOffset;
    }
    public float aftOffset() {
        deserialise();
        return this.aftOffset;
    }

    public void aftMultiplier(float aftMultiplier) {
        deserialise();
        this.aftMultiplier = aftMultiplier;
    }
    public float aftMultiplier() {
        deserialise();
        return this.aftMultiplier;
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

            s.writeFloat(this.foreOffset);
            s.writeFloat(this.foreMultiplier);
            s.writeFloat(this.aftOffset);
            s.writeFloat(this.aftMultiplier);

            return bs.toByteArray();
        }
    }

    public DepthCalibrationMessage(){
        super(80, "control");
        this.bytes = null;
    }

    public DepthCalibrationMessage(Float foreOffset, Float foreMultiplier, Float aftOffset, Float aftMultiplier) {
        super(80, "control");
        this.bytes = null;

        this.foreOffset = foreOffset;
        this.foreMultiplier = foreMultiplier;
        this.aftOffset = aftOffset;
        this.aftMultiplier = aftMultiplier;
    }

    public DepthCalibrationMessage(byte[] bytes) {
        super(80, "control");
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
                    throw new IllegalArgumentException("Attempted to create DepthCalibrationMessage with invalid id");
                }

                this.foreOffset = s.readFloat();
                this.foreMultiplier = s.readFloat();
                this.aftOffset = s.readFloat();
                this.aftMultiplier = s.readFloat();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
