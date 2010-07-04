package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class TelemetryMessage extends Message {
    int m_id = 3;
    protected floatYPR orientation;
    protected float depth;

    private byte[] bytes;

    public void orientation(floatYPR orientation) {
        deserialise();
        this.orientation = orientation;
    }
    public floatYPR orientation() {
        deserialise();
        return this.orientation;
    }

    public void depth(float depth) {
        deserialise();
        this.depth = depth;
    }
    public float depth() {
        deserialise();
        return this.depth;
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

            this.orientation.writeInto(s);
            s.writeFloat(this.depth);

            return bs.toByteArray();
        }
    }

    public TelemetryMessage(){
        super(3, "telemetry");
        this.bytes = null;
    }

    public TelemetryMessage(floatYPR orientation, Float depth) {
        super(3, "telemetry");
        this.bytes = null;

        this.orientation = orientation;
        this.depth = depth;
    }

    public TelemetryMessage(byte[] bytes) {
        super(3, "telemetry");
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
                    throw new IllegalArgumentException("Attempted to create TelemetryMessage with invalid id");
                }

                this.orientation = floatYPR.readFrom(s);
                this.depth = s.readFloat();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
