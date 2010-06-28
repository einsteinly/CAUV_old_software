package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class GraphDescriptionMessage extends Message {
    int m_id = 18;
    public HashMap< Integer, NodeType > nodeTypes;
    public HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs;
    public HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs;
    public HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams;

    public void nodeTypes(HashMap< Integer, NodeType > nodeTypes){
        this.nodeTypes = nodeTypes;
    }
    public HashMap< Integer, NodeType > nodeTypes(){
        return this.nodeTypes;
    }

    public void nodeInputs(HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs){
        this.nodeInputs = nodeInputs;
    }
    public HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs(){
        return this.nodeInputs;
    }

    public void nodeOutputs(HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs){
        this.nodeOutputs = nodeOutputs;
    }
    public HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs(){
        return this.nodeOutputs;
    }

    public void nodeParams(HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams){
        this.nodeParams = nodeParams;
    }
    public HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams(){
        return this.nodeParams;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeLong(this.nodeTypes.size());
        for (Map.Entry<int, NodeType> nodeTypes_i : this.nodeTypes)
        {
            int nodeTypes_key = this.nodeTypes.getKey();
            s.writeInt(nodeTypes_key);
            NodeType nodeTypes_val = this.nodeTypes.getValue();
            nodeTypes_val.writeInto(s);
        }
        s.writeLong(this.nodeInputs.size());
        for (Map.Entry<int, HashMap< String, NodeOutput >> nodeInputs_i : this.nodeInputs)
        {
            int nodeInputs_key = this.nodeInputs.getKey();
            s.writeInt(nodeInputs_key);
            HashMap< String, NodeOutput > nodeInputs_val = this.nodeInputs.getValue();
            s.writeLong(nodeInputs_val.size());
            for (Map.Entry<String, NodeOutput> nodeInputs_val_i : nodeInputs_val)
            {
                String nodeInputs_val_key = nodeInputs_val.getKey();
                s.writeInt(nodeInputs_val_key.length());
                s.writeBytes(nodeInputs_val_key);
                NodeOutput nodeInputs_val_val = nodeInputs_val.getValue();
                nodeInputs_val_val.writeInto(s);
            }
        }
        s.writeLong(this.nodeOutputs.size());
        for (Map.Entry<int, HashMap< String, Vector< NodeInput > >> nodeOutputs_i : this.nodeOutputs)
        {
            int nodeOutputs_key = this.nodeOutputs.getKey();
            s.writeInt(nodeOutputs_key);
            HashMap< String, Vector< NodeInput > > nodeOutputs_val = this.nodeOutputs.getValue();
            s.writeLong(nodeOutputs_val.size());
            for (Map.Entry<String, Vector< NodeInput >> nodeOutputs_val_i : nodeOutputs_val)
            {
                String nodeOutputs_val_key = nodeOutputs_val.getKey();
                s.writeInt(nodeOutputs_val_key.length());
                s.writeBytes(nodeOutputs_val_key);
                Vector< NodeInput > nodeOutputs_val_val = nodeOutputs_val.getValue();
                s.writeLong(nodeOutputs_val_val.size());
                for (int nodeOutputs_val_val_i = 0; nodeOutputs_val_val_i < nodeOutputs_val_val.size(); nodeOutputs_val_val_i++)
                {
                    NodeInput nodeOutputs_val_val_val = nodeOutputs_val_val.get(nodeOutputs_val_val_i);
                    nodeOutputs_val_val_val.writeInto(s);
                }
            }
        }
        s.writeLong(this.nodeParams.size());
        for (Map.Entry<int, HashMap< String, NodeParamValue >> nodeParams_i : this.nodeParams)
        {
            int nodeParams_key = this.nodeParams.getKey();
            s.writeInt(nodeParams_key);
            HashMap< String, NodeParamValue > nodeParams_val = this.nodeParams.getValue();
            s.writeLong(nodeParams_val.size());
            for (Map.Entry<String, NodeParamValue> nodeParams_val_i : nodeParams_val)
            {
                String nodeParams_val_key = nodeParams_val.getKey();
                s.writeInt(nodeParams_val_key.length());
                s.writeBytes(nodeParams_val_key);
                NodeParamValue nodeParams_val_val = nodeParams_val.getValue();
                nodeParams_val_val.writeInto(s);
            }
        }

        return bs.toByteArray();
    }

    public GraphDescriptionMessage(){
        super(18, "pl_gui");
    }

    public GraphDescriptionMessage(HashMap< Integer, NodeType > nodeTypes, HashMap< Integer, HashMap< String, NodeOutput > > nodeInputs, HashMap< Integer, HashMap< String, Vector< NodeInput > > > nodeOutputs, HashMap< Integer, HashMap< String, NodeParamValue > > nodeParams) {

        this.nodeTypes = nodeTypes;
        this.nodeInputs = nodeInputs;
        this.nodeOutputs = nodeOutputs;
        this.nodeParams = nodeParams;
    }

    public GraphDescriptionMessage(byte[] bytes) throws IOException {
        super(18, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create GraphDescriptionMessage with invalid id");
        }

        this.nodeTypes = new HashMap< Integer, NodeType >();
        long nodeTypes_len = s.readLong();
        for (int nodeTypes_i = 0; nodeTypes_i < nodeTypes_len; nodeTypes_i++)
        {
            int nodeTypes_key;
            nodeTypes_key = s.readInt();
            NodeType nodeTypes_val;
            nodeTypes_val.readFrom(s);
            this.nodeTypes.put(nodeTypes_key, nodeTypes_val);
        }
        this.nodeInputs = new HashMap< Integer, HashMap< String, NodeOutput > >();
        long nodeInputs_len = s.readLong();
        for (int nodeInputs_i = 0; nodeInputs_i < nodeInputs_len; nodeInputs_i++)
        {
            int nodeInputs_key;
            nodeInputs_key = s.readInt();
            HashMap< String, NodeOutput > nodeInputs_val;
            nodeInputs_val = new HashMap< String, NodeOutput >();
            long nodeInputs_val_len = s.readLong();
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
                nodeInputs_val_val.readFrom(s);
                nodeInputs_val.put(nodeInputs_val_key, nodeInputs_val_val);
            }
            this.nodeInputs.put(nodeInputs_key, nodeInputs_val);
        }
        this.nodeOutputs = new HashMap< Integer, HashMap< String, Vector< NodeInput > > >();
        long nodeOutputs_len = s.readLong();
        for (int nodeOutputs_i = 0; nodeOutputs_i < nodeOutputs_len; nodeOutputs_i++)
        {
            int nodeOutputs_key;
            nodeOutputs_key = s.readInt();
            HashMap< String, Vector< NodeInput > > nodeOutputs_val;
            nodeOutputs_val = new HashMap< String, Vector< NodeInput > >();
            long nodeOutputs_val_len = s.readLong();
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
                long $(len)s = s.readLong();
                for (int nodeOutputs_val_val_i = 0; nodeOutputs_val_val_i < nodeOutputs_val_val_len; nodeOutputs_val_val_i++)
                {
                    NodeInput nodeOutputs_val_val_val;
                    nodeOutputs_val_val_val.readFrom(s);
                    nodeOutputs_val_val.add(nodeOutputs_val_val_i, nodeOutputs_val_val_val);
                }
                nodeOutputs_val.put(nodeOutputs_val_key, nodeOutputs_val_val);
            }
            this.nodeOutputs.put(nodeOutputs_key, nodeOutputs_val);
        }
        this.nodeParams = new HashMap< Integer, HashMap< String, NodeParamValue > >();
        long nodeParams_len = s.readLong();
        for (int nodeParams_i = 0; nodeParams_i < nodeParams_len; nodeParams_i++)
        {
            int nodeParams_key;
            nodeParams_key = s.readInt();
            HashMap< String, NodeParamValue > nodeParams_val;
            nodeParams_val = new HashMap< String, NodeParamValue >();
            long nodeParams_val_len = s.readLong();
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
                nodeParams_val_val.readFrom(s);
                nodeParams_val.put(nodeParams_val_key, nodeParams_val_val);
            }
            this.nodeParams.put(nodeParams_key, nodeParams_val);
        }
    }
}
