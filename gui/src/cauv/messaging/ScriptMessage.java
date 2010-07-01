package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ScriptMessage extends Message {
    int m_id = 102;
    protected String script;
    protected float timeout;

    private byte[] bytes;

    public void script(String script) {
        deserialise();
        this.script = script;
    }
    public String script() {
        deserialise();
        return this.script;
    }

    public void timeout(float timeout) {
        deserialise();
        this.timeout = timeout;
    }
    public float timeout() {
        deserialise();
        return this.timeout;
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

            s.writeInt(this.script.length());
            s.writeBytes(this.script);
            s.writeFloat(this.timeout);

            return bs.toByteArray();
        }
    }

    public ScriptMessage(){
        super(102, "control");
        this.bytes = null;
    }

    public ScriptMessage(String script, Float timeout) {
        super(102, "control");
        this.bytes = null;

        this.script = script;
        this.timeout = timeout;
    }

    public ScriptMessage(byte[] bytes) {
        super(102, "control");
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
                    throw new IllegalArgumentException("Attempted to create ScriptMessage with invalid id");
                }

                int script_len = s.readInt();
                byte[] script_bytes = new byte[script_len];
                for (int script_i = 0; script_i < script_len; script_i++)
                {
                    script_bytes[script_i] = s.readByte();
                }
                this.script = new String(script_bytes);
                this.timeout = s.readFloat();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
