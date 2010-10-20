package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class GuiImageMessage extends Message {
    int m_id = 124;
    protected int nodeId;
    protected Image image;

    private byte[] bytes;

    public void nodeId(int nodeId) {
        deserialise();
        this.nodeId = nodeId;
    }
    public int nodeId() {
        deserialise();
        return this.nodeId;
    }

    public void image(Image image) {
        deserialise();
        this.image = image;
    }
    public Image image() {
        deserialise();
        return this.image;
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

            s.writeInt(this.nodeId);
            this.image.writeInto(s);

            return bs.toByteArray();
        }
    }

    public GuiImageMessage(){
        super(124, "pl_gui");
        this.bytes = null;
    }

    public GuiImageMessage(Integer nodeId, Image image) {
        super(124, "pl_gui");
        this.bytes = null;

        this.nodeId = nodeId;
        this.image = image;
    }

    public GuiImageMessage(byte[] bytes) {
        super(124, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create GuiImageMessage with invalid id");
                }

                this.nodeId = s.readInt();
                this.image = Image.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
