package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class AddNodeMessage extends Message {
    int m_id = 5;
    protected NodeType nodeType;
    protected Vector< NodeInputArc > parents;
    protected Vector< NodeOutputArc > children;

    private byte[] bytes;

    public void nodeType(NodeType nodeType) {
        deserialise();
        this.nodeType = nodeType;
    }
    public NodeType nodeType() {
        deserialise();
        return this.nodeType;
    }

    public void parents(Vector< NodeInputArc > parents) {
        deserialise();
        this.parents = parents;
    }
    public Vector< NodeInputArc > parents() {
        deserialise();
        return this.parents;
    }

    public void children(Vector< NodeOutputArc > children) {
        deserialise();
        this.children = children;
    }
    public Vector< NodeOutputArc > children() {
        deserialise();
        return this.children;
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

            this.nodeType.writeInto(s);
            s.writeInt(this.parents.size());
            for (int parents_i = 0; parents_i < this.parents.size(); parents_i++)
            {
                NodeInputArc parents_val = this.parents.get(parents_i);
                parents_val.writeInto(s);
            }
            s.writeInt(this.children.size());
            for (int children_i = 0; children_i < this.children.size(); children_i++)
            {
                NodeOutputArc children_val = this.children.get(children_i);
                children_val.writeInto(s);
            }

            return bs.toByteArray();
        }
    }

    public AddNodeMessage(){
        super(5, "pipeline");
        this.bytes = null;
    }

    public AddNodeMessage(NodeType nodeType, Vector< NodeInputArc > parents, Vector< NodeOutputArc > children) {
        super(5, "pipeline");
        this.bytes = null;

        this.nodeType = nodeType;
        this.parents = parents;
        this.children = children;
    }

    public AddNodeMessage(byte[] bytes) {
        super(5, "pipeline");
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
                    throw new IllegalArgumentException("Attempted to create AddNodeMessage with invalid id");
                }

                this.nodeType = NodeType.readFrom(s);
                this.parents = new Vector< NodeInputArc >();
                long parents_len = s.readInt();
                for (int parents_i = 0; parents_i < parents_len; parents_i++)
                {
                    NodeInputArc parents_val;
                    parents_val = NodeInputArc.readFrom(s);
                    this.parents.add(parents_i, parents_val);
                }
                this.children = new Vector< NodeOutputArc >();
                long children_len = s.readInt();
                for (int children_i = 0; children_i < children_len; children_i++)
                {
                    NodeOutputArc children_val;
                    children_val = NodeOutputArc.readFrom(s);
                    this.children.add(children_i, children_val);
                }

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
