package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum ParamType {

    Int32, Float, String, Bool;

    public static ParamType readFrom(LEDataInputStream s) throws IOException {
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

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Int32:
                s.writeByte(0);
                break;
            case Float:
                s.writeByte(1);
                break;
            case String:
                s.writeByte(2);
                break;
            case Bool:
                s.writeByte(3);
                break;
        }
    }
}


