package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class HoughLinesMessage extends Message {
    int m_id = 130;
    protected Vector< Line > lines;

    private byte[] bytes;

    public void lines(Vector< Line > lines) {
        deserialise();
        this.lines = lines;
    }
    public Vector< Line > lines() {
        deserialise();
        return this.lines;
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

            s.writeInt(this.lines.size());
            for (int lines_i = 0; lines_i < this.lines.size(); lines_i++)
            {
                Line lines_val = this.lines.get(lines_i);
                lines_val.writeInto(s);
            }

            return bs.toByteArray();
        }
    }

    public HoughLinesMessage(){
        super(130, "processing");
        this.bytes = null;
    }

    public HoughLinesMessage(Vector< Line > lines) {
        super(130, "processing");
        this.bytes = null;

        this.lines = lines;
    }

    public HoughLinesMessage(byte[] bytes) {
        super(130, "processing");
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
                    throw new IllegalArgumentException("Attempted to create HoughLinesMessage with invalid id");
                }

                this.lines = new Vector< Line >();
                long lines_len = s.readInt();
                for (int lines_i = 0; lines_i < lines_len; lines_i++)
                {
                    Line lines_val;
                    lines_val = Line.readFrom(s);
                    this.lines.add(lines_i, lines_val);
                }

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
