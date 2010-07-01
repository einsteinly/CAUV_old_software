package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class PressureMessage extends Message {
    int m_id = 50;
    protected int fore;
    protected int aft;

    private byte[] bytes;

    public void fore(int fore) {
        deserialise();
        this.fore = fore;
    }
    public int fore() {
        deserialise();
        return this.fore;
    }

    public void aft(int aft) {
        deserialise();
        this.aft = aft;
    }
    public int aft() {
        deserialise();
        return this.aft;
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

            s.writeShort(this.fore);
            s.writeShort(this.aft);

            return bs.toByteArray();
        }
    }

    public PressureMessage(){
        super(50, "pressure");
        this.bytes = null;
    }

    public PressureMessage(Integer fore, Integer aft) {
        super(50, "pressure");
        this.bytes = null;

        this.fore = fore;
        this.aft = aft;
    }

    public PressureMessage(byte[] bytes) {
        super(50, "pressure");
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
                    throw new IllegalArgumentException("Attempted to create PressureMessage with invalid id");
                }

                this.fore = s.readUnsignedShort();
                this.aft = s.readUnsignedShort();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
