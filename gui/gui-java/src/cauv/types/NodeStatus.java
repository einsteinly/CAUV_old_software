package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum NodeStatus {

    AllowQueue, ExecQueued, Executing;

    public static NodeStatus readFrom(LEDataInputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 1:
                return AllowQueue;
            case 2:
                return ExecQueued;
            case 4:
                return Executing;
            default:
                throw new IllegalArgumentException("Unrecognized NodeStatus value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case AllowQueue:
                s.writeByte(1);
                break;
            case ExecQueued:
                s.writeByte(2);
                break;
            case Executing:
                s.writeByte(4);
                break;
        }
    }
}


