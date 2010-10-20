package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class DebugLevelMessage extends Message {
    int m_id = 1;
    protected int level;

    private byte[] bytes;

    public void level(int level) {
        deserialise();
        this.level = level;
    }
    public int level() {
        deserialise();
        return this.level;
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

            s.writeInt(this.level);

            return bs.toByteArray();
        }
    }

    public DebugLevelMessage(){
        super(1, "debug");
        this.bytes = null;
    }

    public DebugLevelMessage(Integer level) {
        super(1, "debug");
        this.bytes = null;

        this.level = level;
    }

    public DebugLevelMessage(byte[] bytes) {
        super(1, "debug");
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
                    throw new IllegalArgumentException("Attempted to create DebugLevelMessage with invalid id");
                }

                this.level = s.readInt();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
