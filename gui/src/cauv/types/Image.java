package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class Image {

    public String format;
    public int loadFlags;
    public Vector<Integer> compressionParams;
    public Vector<Byte> data;
    
    public Image()
    {
    }

    public static Image readFrom(LEDataInputStream s) throws IOException {
        Image val = new Image();
        
        //format
        int msg_len = s.readInt();
        byte[] msg_bytes = new byte[msg_len];
        for (int msg_i = 0; msg_i < msg_len; msg_i++)
        {
            msg_bytes[msg_i] = s.readByte();
        }
        val.format = new String(msg_bytes);
        
        
        // compressionParams
        val.compressionParams = new Vector< Integer >();
        long compressionParams_len = s.readLong();
        System.out.println("Length: " + compressionParams_len);
        for (int compressionParams_i = 0; compressionParams_i < compressionParams_len; compressionParams_i++)
        {
            val.compressionParams.add(compressionParams_i, s.readInt());
        }
        
        /*
        // load flags
        val.loadFlags = s.readInt();
        
        // data
        val.data = new Vector< Byte >();
        data_len = s.readLong();
        for (int data_i = 0; data_i < data_len; data_i++)
        {
            val.data.add(data_i, s.readByte());
        }
        */
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        throw new RuntimeException("Not implemented yet.");
    }
}
