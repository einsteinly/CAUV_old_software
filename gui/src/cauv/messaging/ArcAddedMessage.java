package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ArcAddedMessage extends Message {
    int m_id = 119;
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
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        this.from.writeInto(s);
        this.to.writeInto(s);

        return bs.toByteArray();
    }

    public ArcAddedMessage(){
        super(119, "pl_gui");
    }

    public ArcAddedMessage(NodeOutput from, NodeInput to) {
        super(119, "pl_gui");
        this.from = from;
        this.to = to;
    }

    public ArcAddedMessage(byte[] bytes) throws IOException {
        super(119, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create ArcAddedMessage with invalid id");
        }

        this.from = NodeOutput.readFrom(s);
        this.to = NodeInput.readFrom(s);
    }
}
