package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum NodeStatus {

    AllowQueue, ExecQueued, Executing;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case AllowQueue:
                s.writeByte(1);
            case ExecQueued:
                s.writeByte(2);
            case Executing:
                s.writeByte(4);
        }
    }

    public static NodeStatus readFrom(DataOutputStream s) throws IOException {
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

}


