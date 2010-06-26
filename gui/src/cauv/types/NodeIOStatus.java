package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum NodeIOStatus {

    New, Valid, Demanded;

    public static NodeIOStatus readFrom(LEDataInputStream s) throws IOException {
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

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case New:
                s.writeByte(1);
                break;
            case Valid:
                s.writeByte(2);
                break;
            case Demanded:
                s.writeByte(4);
                break;
        }
    }
}


