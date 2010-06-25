package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum DebugType {

    Trace, Debug, Error;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Trace:
                s.writeByte(0);
            case Debug:
                s.writeByte(1);
            case Error:
                s.writeByte(2);
        }
    }

    public static DebugType readFrom(DataOutputStream s) throws IOException {
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

}


