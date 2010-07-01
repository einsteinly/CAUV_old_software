package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class StateMessage extends Message {
    int m_id = 81;
    protected floatYPR orientation;

    private byte[] bytes;

    public void orientation(floatYPR orientation) {
        deserialise();
        this.orientation = orientation;
    }
    public floatYPR orientation() {
        deserialise();
        return this.orientation;
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

            return bs.toByteArray();
        }
    }

    public StateMessage(){
        super(81, "state");
        this.bytes = null;
    }

    public StateMessage(floatYPR orientation) {
        super(81, "state");
        this.bytes = null;

        this.orientation = orientation;
    }

    public StateMessage(byte[] bytes) {
        super(81, "state");
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
                    throw new IllegalArgumentException("Attempted to create StateMessage with invalid id");
                }

                this.orientation = floatYPR.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
