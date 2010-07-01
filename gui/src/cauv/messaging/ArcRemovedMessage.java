package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class ArcRemovedMessage extends Message {
    int m_id = 120;
    protected NodeOutput from;
    protected NodeInput to;

    private byte[] bytes;

    public void from(NodeOutput from) {
        deserialise();
        this.from = from;
    }
    public NodeOutput from() {
        deserialise();
        return this.from;
    }

    public void to(NodeInput to) {
        deserialise();
        this.to = to;
    }
    public NodeInput to() {
        deserialise();
        return this.to;
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

            this.from.writeInto(s);
            this.to.writeInto(s);

            return bs.toByteArray();
        }
    }

    public ArcRemovedMessage(){
        super(120, "pl_gui");
        this.bytes = null;
    }

    public ArcRemovedMessage(NodeOutput from, NodeInput to) {
        super(120, "pl_gui");
        this.bytes = null;

        this.from = from;
        this.to = to;
    }

    public ArcRemovedMessage(byte[] bytes) {
        super(120, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create ArcRemovedMessage with invalid id");
                }

                this.from = NodeOutput.readFrom(s);
                this.to = NodeInput.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
