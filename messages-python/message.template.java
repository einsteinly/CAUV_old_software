package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class ${name}Message extends Message {
    int m_id = ${id};
    #for $f in $fields
    public $toJavaType($f.type) $f.name;
    #end for

    #for $f in $fields
    public void ${f.name}($toJavaType(f.type) ${f.name}){
        this.${f.name} = ${f.name};
    }

    public $toJavaType(f.type) ${f.name}(){
        return this.${f.name};
    }

    #end for

    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        #for $f in $fields
	$serialiseJavaType(f.type, f.name, 0, "this.")
	#end for

        return bs.toByteArray();
    }

    public ${name}Message(){
        super(${id}, "${group.name}");
    }

    public ${name}Message(#slurp
                          #for i, f in $enumerate($fields)
#*                       *#$toJavaType($f.type) $f.name#if $i < $len($fields) - 1#, #end if##slurp
                          #end for
#*                       *#) {

        #for $f in $fields
	this.${f.name} = ${f.name};
        #end for
    }

    public ${name}Message(byte[] bytes) throws IOException {
        super(${id}, "${group.name}");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id) { throw new IllegalArgumentException(
                "Attempted to create ${name}Message with invalid id"); }

	#for $f in $fields
	$deserialiseJavaType(f.type, f.name, 0, "this.")
	#end for
    }
}
