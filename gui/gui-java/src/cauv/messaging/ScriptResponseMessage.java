package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ScriptResponseMessage extends Message {
    int m_id = 103;
    protected String response;

    private byte[] bytes;

    public void response(String response) {
        deserialise();
        this.response = response;
    }
    public String response() {
        deserialise();
        return this.response;
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

            s.writeInt(this.response.length());
            s.writeBytes(this.response);

            return bs.toByteArray();
        }
    }

    public ScriptResponseMessage(){
        super(103, "gui");
        this.bytes = null;
    }

    public ScriptResponseMessage(String response) {
        super(103, "gui");
        this.bytes = null;

        this.response = response;
    }

    public ScriptResponseMessage(byte[] bytes) {
        super(103, "gui");
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
                    throw new IllegalArgumentException("Attempted to create ScriptResponseMessage with invalid id");
                }

                int response_len = s.readInt();
                byte[] response_bytes = new byte[response_len];
                for (int response_i = 0; response_i < response_len; response_i++)
                {
                    response_bytes[response_i] = s.readByte();
                }
                this.response = new String(response_bytes);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
