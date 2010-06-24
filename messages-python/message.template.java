package $package;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import ${rootpackage}.types.*;
import ${rootpackage}.utils.*;

public class ${m.name}Message extends Message {
    int m_id = $m.id;
    #for $f in $m.fields
    public $toJavaType($f.type) $f.name;
    #end for

    #for $f in $m.fields
    public void ${f.name}($toJavaType(f.type) ${f.name}){
        this.${f.name} = ${f.name};
    }
    public $toJavaType(f.type) ${f.name}(){
        return this.${f.name};
    }

    #end for

    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        #for $f in $m.fields
$serialiseJavaType(f.type, f.name, 2, "this.")
        #end for

        return bs.toByteArray();
    }

    public ${m.name}Message(){
        super($m.id, "${group.name}");
    }

    #if $len($m.fields) > 0
    public ${m.name}Message(#slurp
                            #for i, f in $enumerate($m.fields)
#*                         *#$toJavaType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                            #end for
#*                         *#) {
        super($m.id, "${group.name}");
        #for $f in $m.fields
        this.${f.name} = ${f.name};
        #end for
    }
    #end if

    public ${m.name}Message(byte[] bytes) throws IOException {
        super(${m.id}, "${group.name}");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create ${m.name}Message with invalid id");
        }

        #for $f in $m.fields
$deserialiseJavaType(f.type, f.name, 2, "this.")
        #end for
    }
}
