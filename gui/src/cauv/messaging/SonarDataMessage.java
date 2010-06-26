package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class SonarDataMessage extends Message {
    int m_id = 30;
    public SonarDataLine line;

    public void line(SonarDataLine line){
        this.line = line;
    }
    public SonarDataLine line(){
        return this.line;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        this.line.writeInto(s);

        return bs.toByteArray();
    }

    public SonarDataMessage(){
        super(30, "sonarout");
    }

    public SonarDataMessage(SonarDataLine line) {
        super(30, "sonarout");
        this.line = line;
    }

    public SonarDataMessage(byte[] bytes) throws IOException {
        super(30, "sonarout");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create SonarDataMessage with invalid id");
        }

        this.line = SonarDataLine.readFrom(s);
    }
}
