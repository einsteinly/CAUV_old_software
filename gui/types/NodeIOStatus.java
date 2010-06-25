package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum NodeIOStatus {

    New, Valid, Demanded;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case New:
                s.writeByte(1);
            case Valid:
                s.writeByte(2);
            case Demanded:
                s.writeByte(4);
        }
    }

    public static NodeIOStatus readFrom(DataOutputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 1:
                return New;
            case 2:
                return Valid;
            case 4:
                return Demanded;
            default:
                throw new IllegalArgumentException("Unrecognized NodeIOStatus value: " + val);
        }
    }

}


