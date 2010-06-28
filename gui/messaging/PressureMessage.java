package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class PressureMessage extends Message {
    int m_id = 50;
    public int fore;
    public int aft;

    public void fore(int fore){
        this.fore = fore;
    }
    public int fore(){
        return this.fore;
    }

    public void aft(int aft){
        this.aft = aft;
    }
    public int aft(){
        return this.aft;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeShort(this.fore);
        s.writeShort(this.aft);

        return bs.toByteArray();
    }

    public PressureMessage(){
        super(50, "pressure");
    }

    public PressureMessage(int fore, int aft) {

        this.fore = fore;
        this.aft = aft;
    }

    public PressureMessage(byte[] bytes) throws IOException {
        super(50, "pressure");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create PressureMessage with invalid id");
        }

        this.fore = s.readUnsignedShort();
        this.aft = s.readUnsignedShort();
    }
}
