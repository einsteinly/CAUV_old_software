package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import ${rootpackage}.utils.*; 

public enum $e.name {

    #for i, v in $enumerate($e.values)#${v.name}#if $i < $len($e.values) - 1#, #end if##end for#;

    public static $e.name readFrom(LEDataInputStream s) throws IOException {
        $toJavaType($e.type) $deserialiseJavaType($e.type, "val")
        switch (val) {
            #for $v in $e.values
            case $v.value:
                return $v.name;
            #end for            
            default:
                throw new IllegalArgumentException("Unrecognized ${e.name} value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            #for $v in $e.values
            case $v.name:
                $serialiseJavaType($e.type, $str($v.value))
            #end for
        }
    }
}


