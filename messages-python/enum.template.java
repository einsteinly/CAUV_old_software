package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum $e.name {

    #for i, val in $enumerate($e.values)
${val.name}#if $i < $len($e.values) - 1#, #end if##slurp
#end for
;

#set $type = $e.type


    public void writeInto(DataInputStream s) throws IOException {
        switch (this) {
            #for $val in $e.values
            case $val.name:
                s.write${toJavaType(type, 1)}(${val.value});
            #end for
        }
    }

    public static $e.name readFrom(DataInputStream s) throws IOException {
        int val = s.read${toJavaType(type, 1)}(); 
        switch (val) {
            #for $val in $e.values
            case $val.value:
                return $val.name;
            #end for            
            default:
                throw new IllegalArgumentException("Unrecognized ${e.name} value: " + val);
        }
    }

}


