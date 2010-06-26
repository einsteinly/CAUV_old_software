package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class AddArcMessage extends Message {
    int m_id = 9;
    public NodeOutput from;
    public NodeInput to;

    public void from(NodeOutput from){
        this.from = from;
    }
    public NodeOutput from(){
        return this.from;
    }

    public void to(NodeInput to){
        this.to = to;
    }
    public NodeInput to(){
        return this.to;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        this.from.writeInto(s);
        this.to.writeInto(s);

        return bs.toByteArray();
    }

    public AddArcMessage(){
        super(9, "pipeline");
    }

    public AddArcMessage(NodeOutput from, NodeInput to) {

        this.from = from;
        this.to = to;
    }

    public AddArcMessage(byte[] bytes) throws IOException {
        super(9, "pipeline");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create AddArcMessage with invalid id");
        }

        this.from.readFrom(s);
        this.to.readFrom(s);
    }
}
