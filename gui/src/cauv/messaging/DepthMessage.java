package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class DepthMessage extends Message {
    int m_id = 51;
    public float depth;

    public void depth(float depth){
        this.depth = depth;
    }
    public float depth(){
        return this.depth;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeFloat(this.depth);

        return bs.toByteArray();
    }

    public DepthMessage(){
        super(51, "telemetry");
    }

    public DepthMessage(Float depth) {
        super(51, "telemetry");
        this.depth = depth;
    }

    public DepthMessage(byte[] bytes) throws IOException {
        super(51, "telemetry");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create DepthMessage with invalid id");
        }

        this.depth = s.readFloat();
    }
}
