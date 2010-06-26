package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class DebugMessage extends Message {
    int m_id = 0;
    public DebugType type;
    public String msg;

    public void type(DebugType type){
        this.type = type;
    }
    public DebugType type(){
        return this.type;
    }

    public void msg(String msg){
        this.msg = msg;
    }
    public String msg(){
        return this.msg;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        this.type.writeInto(s);
        s.writeInt(this.msg.length());
        s.writeBytes(this.msg);

        return bs.toByteArray();
    }

    public DebugMessage(){
        super(0, "debug");
    }

    public DebugMessage(DebugType type, String msg) {

        this.type = type;
        this.msg = msg;
    }

    public DebugMessage(byte[] bytes) throws IOException {
        super(0, "debug");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create DebugMessage with invalid id");
        }

        this.type.readFrom(s);
        int msg_len = s.readInt();
        byte[] msg_bytes = new byte[msg_len];
        for (int msg_i = 0; msg_i < msg_len; msg_i++)
        {
            msg_bytes[msg_i] = s.readByte();
        }
        this.msg = new String(msg_bytes);
    }
}
