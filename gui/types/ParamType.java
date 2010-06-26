package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum ParamType {

    Int32, Float, String, Bool;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Int32:
                s.writeByte(0);
            case Float:
                s.writeByte(1);
            case String:
                s.writeByte(2);
            case Bool:
                s.writeByte(3);
        }
    }

    public static ParamType readFrom(DataOutputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 0:
                return Int32;
            case 1:
                return Float;
            case 2:
                return String;
            case 3:
                return Bool;
            default:
                throw new IllegalArgumentException("Unrecognized ParamType value: " + val);
        }
    }

}


