package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class AIMessage extends Message {
    int m_id = 200;
    protected String msg;

    private byte[] bytes;

    public void msg(String msg) {
        deserialise();
        this.msg = msg;
    }
    public String msg() {
        deserialise();
        return this.msg;
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

            s.writeInt(this.msg.length());
            s.writeBytes(this.msg);

            return bs.toByteArray();
        }
    }

    public AIMessage(){
        super(200, "ai");
        this.bytes = null;
    }

    public AIMessage(String msg) {
        super(200, "ai");
        this.bytes = null;

        this.msg = msg;
    }

    public AIMessage(byte[] bytes) {
        super(200, "ai");
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
                    throw new IllegalArgumentException("Attempted to create AIMessage with invalid id");
                }

                int msg_len = s.readInt();
                byte[] msg_bytes = new byte[msg_len];
                for (int msg_i = 0; msg_i < msg_len; msg_i++)
                {
                    msg_bytes[msg_i] = s.readByte();
                }
                this.msg = new String(msg_bytes);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
