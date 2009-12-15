package cauv.auv;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Vector;

import spread.AdvancedMessageListener;
import spread.SpreadConnection;
import spread.SpreadException;
import spread.SpreadGroup;
import spread.SpreadMessage;

public class MessageSocket extends MessageSource implements AdvancedMessageListener {

    private static final boolean DEBUG = true;
    Vector<SpreadGroup> groups = new Vector<SpreadGroup>();
    Vector<MembershipObserver> m_membership_observers = new Vector<MembershipObserver>();

    public interface ConnectionStateObserver {
        public void onConnect(MessageSocket connection);
        public void onDisconnect(MessageSocket connection);
    }
    public interface MembershipObserver {
        public void onMembershipChanged(SpreadGroup [] members);
    }
    public SpreadConnection m_connection;
    private Vector<ConnectionStateObserver> m_connection_state_listeners = new Vector<ConnectionStateObserver>();

    public MessageSocket(String address, int port) throws UnknownHostException, IOException {
        //
        // set up the spread connection.
        // connect to the spread daemon running on the AUV

        if (!DEBUG) {
            m_connection = new SpreadConnection();
            m_connection.add(this);

            try { // register for received messages m_connection.add(this);
                m_connection.connect(InetAddress.getByName(address), port, "GUI", false, false);
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

    public void disconnect() throws IOException {
        if (!DEBUG) {
            try {
                m_connection.disconnect();
                m_connection.remove(this);
            } catch (SpreadException ex) {
                throw new IOException(ex.getMessage());
            }
        }

        for (ConnectionStateObserver c : m_connection_state_listeners) {
            c.onDisconnect(this);
        }
    }

    public void joinGroup(String name) throws IOException {
        if (!DEBUG) {
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

    public void removeConnectionStateListener(ConnectionStateObserver listener) {
        this.m_connection_state_listeners.remove(listener);
    }

    public void sendMessage(Message m) throws IOException {

        System.out.println("Sending " + m);

        if (!DEBUG) {
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
            o.onMembershipChanged(message.getMembershipInfo().getMembers());
    }

    @Override
    public void regularMessageReceived(SpreadMessage message) {
        // pass the message onto our message factory to convert it to a CAUV
        // message
        notifyObservers(message.getData());
    }
}
