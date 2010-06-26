package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class TelemetryMessage extends Message {
    int m_id = 3;
    public floatYPR orientation;

    public void orientation(floatYPR orientation){
        this.orientation = orientation;
    }
    public floatYPR orientation(){
        return this.orientation;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        this.orientation.writeInto(s);

        return bs.toByteArray();
    }

    public TelemetryMessage(){
        super(3, "telemetry");
    }

    public TelemetryMessage(floatYPR orientation) {

        this.orientation = orientation;
    }

    public TelemetryMessage(byte[] bytes) throws IOException {
        super(3, "telemetry");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create TelemetryMessage with invalid id");
        }

        this.orientation.readFrom(s);
    }
}
