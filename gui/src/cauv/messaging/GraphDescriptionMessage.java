package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class GraphDescriptionMessage extends Message {
    int m_id = 118;
    protected HashMap< Integer, NodeType > nodeTypes;
    protected HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs;
    protected HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs;
    protected HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams;

    private byte[] bytes;

    public void nodeTypes(HashMap< Integer, NodeType > nodeTypes) {
        deserialise();
        this.nodeTypes = nodeTypes;
    }
    public HashMap< Integer, NodeType > nodeTypes() {
        deserialise();
        return this.nodeTypes;
    }

    public void nodeInputs(HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs) {
        deserialise();
        this.nodeInputs = nodeInputs;
    }
    public HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs() {
        deserialise();
        return this.nodeInputs;
    }

    public void nodeOutputs(HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs) {
        deserialise();
        this.nodeOutputs = nodeOutputs;
    }
    public HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs() {
        deserialise();
        return this.nodeOutputs;
    }

    public void nodeParams(HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams) {
        deserialise();
        this.nodeParams = nodeParams;
    }
    public HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams() {
        deserialise();
        return this.nodeParams;
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

            s.writeInt(this.nodeTypes.size());
            for (Map.Entry<Integer, NodeType> nodeTypes_i : this.nodeTypes.entrySet())
            {
                Integer nodeTypes_key = nodeTypes_i.getKey();
                s.writeInt(nodeTypes_key);
                NodeType nodeTypes_val = nodeTypes_i.getValue();
                nodeTypes_val.writeInto(s);
            }
            s.writeInt(this.nodeInputs.size());
            for (Map.Entry<Integer, HashMap< String, NodeOutput >> nodeInputs_i : this.nodeInputs.entrySet())
            {
                Integer nodeInputs_key = nodeInputs_i.getKey();
                s.writeInt(nodeInputs_key);
                HashMap< String, NodeOutput > nodeInputs_val = nodeInputs_i.getValue();
                s.writeInt(nodeInputs_val.size());
                for (Map.Entry<String, NodeOutput> nodeInputs_val_i : nodeInputs_val.entrySet())
                {
                    String nodeInputs_val_key = nodeInputs_val_i.getKey();
                    s.writeInt(nodeInputs_val_key.length());
                    s.writeBytes(nodeInputs_val_key);
                    NodeOutput nodeInputs_val_val = nodeInputs_val_i.getValue();
                    nodeInputs_val_val.writeInto(s);
                }
            }
            s.writeInt(this.nodeOutputs.size());
            for (Map.Entry<Integer, HashMap< String, Vector< NodeInput > >> nodeOutputs_i : this.nodeOutputs.entrySet())
            {
                Integer nodeOutputs_key = nodeOutputs_i.getKey();
                s.writeInt(nodeOutputs_key);
                HashMap< String, Vector< NodeInput > > nodeOutputs_val = nodeOutputs_i.getValue();
                s.writeInt(nodeOutputs_val.size());
                for (Map.Entry<String, Vector< NodeInput >> nodeOutputs_val_i : nodeOutputs_val.entrySet())
                {
                    String nodeOutputs_val_key = nodeOutputs_val_i.getKey();
                    s.writeInt(nodeOutputs_val_key.length());
                    s.writeBytes(nodeOutputs_val_key);
                    Vector< NodeInput > nodeOutputs_val_val = nodeOutputs_val_i.getValue();
                    s.writeInt(nodeOutputs_val_val.size());
                    for (int nodeOutputs_val_val_i = 0; nodeOutputs_val_val_i < nodeOutputs_val_val.size(); nodeOutputs_val_val_i++)
                    {
                        NodeInput nodeOutputs_val_val_val = nodeOutputs_val_val.get(nodeOutputs_val_val_i);
                        nodeOutputs_val_val_val.writeInto(s);
                    }
                }
            }
            s.writeInt(this.nodeParams.size());
            for (Map.Entry<Integer, HashMap< String, NodeParamValue >> nodeParams_i : this.nodeParams.entrySet())
            {
                Integer nodeParams_key = nodeParams_i.getKey();
                s.writeInt(nodeParams_key);
                HashMap< String, NodeParamValue > nodeParams_val = nodeParams_i.getValue();
                s.writeInt(nodeParams_val.size());
                for (Map.Entry<String, NodeParamValue> nodeParams_val_i : nodeParams_val.entrySet())
                {
                    String nodeParams_val_key = nodeParams_val_i.getKey();
                    s.writeInt(nodeParams_val_key.length());
                    s.writeBytes(nodeParams_val_key);
                    NodeParamValue nodeParams_val_val = nodeParams_val_i.getValue();
                    nodeParams_val_val.writeInto(s);
                }
            }

            return bs.toByteArray();
        }
    }

    public GraphDescriptionMessage(){
        super(118, "pl_gui");
        this.bytes = null;
    }

    public GraphDescriptionMessage(HashMap< Integer, NodeType > nodeTypes, HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs, HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs, HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams) {
        super(118, "pl_gui");
        this.bytes = null;

        this.nodeTypes = nodeTypes;
        this.nodeInputs = nodeInputs;
        this.nodeOutputs = nodeOutputs;
        this.nodeParams = nodeParams;
    }

    public GraphDescriptionMessage(byte[] bytes) {
        super(118, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create GraphDescriptionMessage with invalid id");
                }

                this.nodeTypes = new HashMap< Integer, NodeType >();
                long nodeTypes_len = s.readInt();
                for (int nodeTypes_i = 0; nodeTypes_i < nodeTypes_len; nodeTypes_i++)
                {
                    int nodeTypes_key;
                    nodeTypes_key = s.readInt();
                    NodeType nodeTypes_val;
                    nodeTypes_val = NodeType.readFrom(s);
                    this.nodeTypes.put(nodeTypes_key, nodeTypes_val);
                }
                this.nodeInputs = new HashMap< Integer, HashMap< String, NodeOutput > >();
                long nodeInputs_len = s.readInt();
                for (int nodeInputs_i = 0; nodeInputs_i < nodeInputs_len; nodeInputs_i++)
                {
                    int nodeInputs_key;
                    nodeInputs_key = s.readInt();
                    HashMap< String, NodeOutput > nodeInputs_val;
                    nodeInputs_val = new HashMap< String, NodeOutput >();
                    long nodeInputs_val_len = s.readInt();
                    for (int nodeInputs_val_i = 0; nodeInputs_val_i < nodeInputs_val_len; nodeInputs_val_i++)
                    {
                        String nodeInputs_val_key;
                        int nodeInputs_val_key_len = s.readInt();
                        byte[] nodeInputs_val_key_bytes = new byte[nodeInputs_val_key_len];
                        for (int nodeInputs_val_key_i = 0; nodeInputs_val_key_i < nodeInputs_val_key_len; nodeInputs_val_key_i++)
                        {
                            nodeInputs_val_key_bytes[nodeInputs_val_key_i] = s.readByte();
                        }
                        nodeInputs_val_key = new String(nodeInputs_val_key_bytes);
                        NodeOutput nodeInputs_val_val;
                        nodeInputs_val_val = NodeOutput.readFrom(s);
                        nodeInputs_val.put(nodeInputs_val_key, nodeInputs_val_val);
                    }
                    this.nodeInputs.put(nodeInputs_key, nodeInputs_val);
                }
                this.nodeOutputs = new HashMap< Integer, HashMap< String, Vector< NodeInput > > >();
                long nodeOutputs_len = s.readInt();
                for (int nodeOutputs_i = 0; nodeOutputs_i < nodeOutputs_len; nodeOutputs_i++)
                {
                    int nodeOutputs_key;
                    nodeOutputs_key = s.readInt();
                    HashMap< String, Vector< NodeInput > > nodeOutputs_val;
                    nodeOutputs_val = new HashMap< String, Vector< NodeInput > >();
                    long nodeOutputs_val_len = s.readInt();
                    for (int nodeOutputs_val_i = 0; nodeOutputs_val_i < nodeOutputs_val_len; nodeOutputs_val_i++)
                    {
                        String nodeOutputs_val_key;
                        int nodeOutputs_val_key_len = s.readInt();
                        byte[] nodeOutputs_val_key_bytes = new byte[nodeOutputs_val_key_len];
                        for (int nodeOutputs_val_key_i = 0; nodeOutputs_val_key_i < nodeOutputs_val_key_len; nodeOutputs_val_key_i++)
                        {
                            nodeOutputs_val_key_bytes[nodeOutputs_val_key_i] = s.readByte();
                        }
                        nodeOutputs_val_key = new String(nodeOutputs_val_key_bytes);
                        Vector< NodeInput > nodeOutputs_val_val;
                        nodeOutputs_val_val = new Vector< NodeInput >();
                        long nodeOutputs_val_val_len = s.readInt();
                        for (int nodeOutputs_val_val_i = 0; nodeOutputs_val_val_i < nodeOutputs_val_val_len; nodeOutputs_val_val_i++)
                        {
                            NodeInput nodeOutputs_val_val_val;
                            nodeOutputs_val_val_val = NodeInput.readFrom(s);
                            nodeOutputs_val_val.add(nodeOutputs_val_val_i, nodeOutputs_val_val_val);
                        }
                        nodeOutputs_val.put(nodeOutputs_val_key, nodeOutputs_val_val);
                    }
                    this.nodeOutputs.put(nodeOutputs_key, nodeOutputs_val);
                }
                this.nodeParams = new HashMap< Integer, HashMap< String, NodeParamValue > >();
                long nodeParams_len = s.readInt();
                for (int nodeParams_i = 0; nodeParams_i < nodeParams_len; nodeParams_i++)
                {
                    int nodeParams_key;
                    nodeParams_key = s.readInt();
                    HashMap< String, NodeParamValue > nodeParams_val;
                    nodeParams_val = new HashMap< String, NodeParamValue >();
                    long nodeParams_val_len = s.readInt();
                    for (int nodeParams_val_i = 0; nodeParams_val_i < nodeParams_val_len; nodeParams_val_i++)
                    {
                        String nodeParams_val_key;
                        int nodeParams_val_key_len = s.readInt();
                        byte[] nodeParams_val_key_bytes = new byte[nodeParams_val_key_len];
                        for (int nodeParams_val_key_i = 0; nodeParams_val_key_i < nodeParams_val_key_len; nodeParams_val_key_i++)
                        {
                            nodeParams_val_key_bytes[nodeParams_val_key_i] = s.readByte();
                        }
                        nodeParams_val_key = new String(nodeParams_val_key_bytes);
                        NodeParamValue nodeParams_val_val;
                        nodeParams_val_val = NodeParamValue.readFrom(s);
                        nodeParams_val.put(nodeParams_val_key, nodeParams_val_val);
                    }
                    this.nodeParams.put(nodeParams_key, nodeParams_val);
                }

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}