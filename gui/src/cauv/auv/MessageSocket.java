package cauv.auv;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Vector;

import cauv.messaging.MessageSource;
import cauv.messaging.Message;

import spread.AdvancedMessageListener;
import spread.SpreadConnection;
import spread.SpreadException;
import spread.SpreadGroup;
import spread.SpreadMessage;

public class MessageSocket extends MessageSource implements AdvancedMessageListener {

    protected static boolean debug = false;
    protected boolean enabled = true;
    Vector<SpreadGroup> groups = new Vector<SpreadGroup>();
    Vector<MembershipObserver> m_membership_observers = new Vector<MembershipObserver>();
    
    public interface ConnectionStateObserver {
        public void onConnect(MessageSocket connection);
        public void onDisconnect(MessageSocket connection);
    }
    public interface MembershipObserver {
        public void onMembershipChanged(SpreadMessage message);
    }
    public SpreadConnection m_connection;
    private Vector<ConnectionStateObserver> m_connection_state_listeners = new Vector<ConnectionStateObserver>();

    public MessageSocket(String address, int port, String name) throws UnknownHostException, IOException {
        //
        // set up the spread connection.
        // connect to the spread daemon running on the AUV
        
        if (!debug) {
            m_connection = new SpreadConnection();
            m_connection.add(this);

            try {
            	// register for received messages
            	m_connection.add(this);
                m_connection.connect(InetAddress.getByName(address), port, name, false, true);
            } catch (SpreadException ex) {
                for (ConnectionStateObserver c : m_connection_state_listeners) {
                    c.onDisconnect(this);
                }
                throw new IOException(ex.getMessage());
            }
        }

        for (ConnectionStateObserver c : m_connection_state_listeners) {
            c.onConnect(this);
        }
    }

    public static boolean getDebug(){
        return debug;
    }
    
    public static void setDebug(boolean state){
        debug = state;
    }
    
    public void setEnabled(boolean state){
        this.enabled = state;
    }
    
    public void disconnect() throws IOException {
        if (!debug) {
            try {
                //m_connection.remove(this);
                if(m_connection != null)
                    m_connection.disconnect();
            } catch (SpreadException ex) {
                throw new IOException(ex.getMessage());
            }
        }

        for (ConnectionStateObserver c : m_connection_state_listeners) {
            c.onDisconnect(this);
        }
    }

    public void joinGroup(String name) throws IOException {
        if (!debug) {
            try {
                SpreadGroup g = new SpreadGroup();
                g.join(m_connection, name);
                groups.add(g);
            } catch (SpreadException ex) {
                throw new IOException(ex.getMessage());
            }
        }
    }

    public Vector<SpreadGroup> getGroups() {
        return groups;
    }

    public void addConnectionStateObserver(ConnectionStateObserver listener) {
        this.m_connection_state_listeners.add(listener);
        listener.onConnect(this);
    }

    public void removeConnectionStateObserver(ConnectionStateObserver listener) {
        this.m_connection_state_listeners.remove(listener);
    }

    public void addMembershipObserver(MembershipObserver obs){
        this.m_membership_observers.add(obs);
    }
    
    public void removeMembershipObserver(MembershipObserver obs) {
        this.m_membership_observers.remove(obs);
    }
    
    public void sendMessage(Message m) throws IOException {

        System.out.println("Sending " + m);

        if(debug) return;
        
        if (this.enabled) {
            SpreadMessage spreadMessage = new SpreadMessage();
            spreadMessage.setData(m.toBytes());
            spreadMessage.addGroup(m.group());
            spreadMessage.setReliable();

            try {
                m_connection.multicast(spreadMessage);
            } catch (SpreadException ex) {
                throw new IOException(ex.getMessage());
            }
        }
    }

    @Override
    public void membershipMessageReceived(SpreadMessage message) {
        for(MembershipObserver o : m_membership_observers)
            o.onMembershipChanged(message);
    }

    @Override
    public void regularMessageReceived(SpreadMessage message) {
        // pass the message onto our message factory to convert it to a CAUV
        // message
        notifyObservers(message.getData());
    }
}
