package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum OutputType {

    Image, Parameter;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Image:
                s.writeByte(0);
            case Parameter:
                s.writeByte(1);
        }
    }

    public static OutputType readFrom(DataOutputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 0:
                return Image;
            case 1:
                return Parameter;
            default:
                throw new IllegalArgumentException("Unrecognized OutputType value: " + val);
        }
    }

}


