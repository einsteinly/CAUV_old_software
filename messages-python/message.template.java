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
    protected $toJavaType($f.type) $f.name;
    #end for

    private byte[] bytes;

    #for $f in $m.fields
    public void ${f.name}($toJavaType(f.type) ${f.name}) {
        deserialise();
        this.${f.name} = ${f.name};
    }
    public $toJavaType(f.type) ${f.name}() {
        deserialise();
        return this.${f.name};
    }

    #end for

    public byte[] toBytes() throws IOException {
        if (bytes != null)
        {
            return bytes;
        }
        else
        {
            ByteArrayOutputStream bs = new ByteArrayOutputStream();
            LEDataOutputStream s = new LEDataOutputStream(bs);
            s.writeInt(m_id);

            #for $f in $m.fields
$serialiseJavaType(f.type, f.name, 3, "this.")
            #end for

            return bs.toByteArray();
        }
    }

    public ${m.name}Message(){
        super($m.id, "${group.name}");
        this.bytes = null;
    }

    #if $len($m.fields) > 0
    public ${m.name}Message(#slurp
                            #for i, f in $enumerate($m.fields)
#*                         *#$toJavaType($f.type, 1) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                            #end for
#*                         *#) {
        super($m.id, "${group.name}");
        this.bytes = null;

        #for $f in $m.fields
        this.${f.name} = ${f.name};
        #end for
    }
    #end if

    public ${m.name}Message(byte[] bytes) {
        super(${m.id}, "${group.name}");
        this.bytes = bytes;
    }

    public void deserialise() {
        try { 
            if (bytes != null)
            {
                ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
                LEDataInputStream s = new LEDataInputStream(bs);
                int buf_id = s.readInt();
                if (buf_id != m_id)
                {
                    throw new IllegalArgumentException("Attempted to create ${m.name}Message with invalid id");
                }

                #for $f in $m.fields
$deserialiseJavaType(f.type, f.name, 4, "this.")
                #end for

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
