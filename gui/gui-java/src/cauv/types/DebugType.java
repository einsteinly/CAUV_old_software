package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum DebugType {

    Trace, Debug, Error;

    public static DebugType readFrom(LEDataInputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 0:
                return Trace;
            case 1:
                return Debug;
            case 2:
                return Error;
            default:
                throw new IllegalArgumentException("Unrecognized DebugType value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Trace:
                s.writeByte(0);
                break;
            case Debug:
                s.writeByte(1);
                break;
            case Error:
                s.writeByte(2);
                break;
        }
    }
}


