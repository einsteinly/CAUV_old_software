package $package;

import java.util.LinkedList;
import java.io.*;

public class MessageSource
{
    protected LinkedList<MessageObserver> m_obs = new LinkedList<MessageObserver>();

    protected MessageSource() {
    }

    public void notifyObservers(byte[] b) {
        if (b.length < 4)
            throw new IllegalArgumentException("Buffer too small to contain message id");

        int id = b[3] << 24 | b[2] << 16 | b[1] << 8 | b[0];
        try {
            switch (id) {
                #for $g in $groups
                #for $m in $g.messages
                #set $className = $m.name + "Message"
                case $m.id: {
                    $className m = new ${className}(b);
                    for (MessageObserver o : m_obs) {
                        o.on${className}(m);
                    }
                    break;
                }
                #end for
                #end for
            }
        } catch (IOException e) {
        }
    }

    public void addObserver(MessageObserver o) {
        m_obs.add(o);
    }

    public void removeObserver(MessageObserver o) {
        m_obs.remove(o);
    }

    public void clearObservers() {
        m_obs.clear();
    }
}
