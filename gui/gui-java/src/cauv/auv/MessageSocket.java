package cauv.auv;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.sql.Time;
import java.util.Vector;

import cauv.messaging.MessageSource;
import cauv.messaging.Message;

import spread.AdvancedMessageListener;
import spread.SpreadConnection;
import spread.SpreadException;
import spread.SpreadGroup;
import spread.SpreadMessage;

public class MessageSocket extends MessageSource {

    protected Vector<byte[]> messages = new Vector<byte[]>();

    protected static boolean debug = false;
    protected boolean enabled = true;
    Vector<SpreadGroup> groups = new Vector<SpreadGroup>();
    Vector<MembershipObserver> m_membership_observers = new Vector<MembershipObserver>();
    protected volatile boolean connected = false;

    public interface ConnectionStateObserver {
        public void onConnect(MessageSocket connection);

        public void onDisconnect(MessageSocket connection);
    }

    public interface MembershipObserver {
        public void onMembershipChanged(SpreadMessage message);
    }

    public SpreadConnection m_connection;
    private Vector<ConnectionStateObserver> m_connection_state_listeners = new Vector<ConnectionStateObserver>();

    public MessageSocket(String address, int port, String name) throws UnknownHostException,
            IOException {

        System.out.println("trying to connect");

        if (debug) {
            this.connected = true;
        } else {
            m_connection = new SpreadConnection();

            try {
                // register for received messages
                m_connection.connect(InetAddress.getByName(address), port, name, true, true);
                this.connected = true;

                // reading thread
                final Thread receieve = new Thread() {

                    public void getMessages() throws InterruptedIOException, SpreadException {
                        SpreadMessage m = m_connection.receive();

                        //System.out.println("getting messages");

                        // ignore messsages from ourself
                        if (m.getSender().equals(m_connection.getPrivateGroup())) return;

                        if (m.isMembership()) {
                            for (MembershipObserver o : m_membership_observers)
                                o.onMembershipChanged(m);
                        } else {
                            messages.add(m.getData());
                        }
                    }

                    public void run() {
                        System.out.println("receieve thread started");

                        while (connected) {
                            try {
                                getMessages();

                                // handle remaining messages in queue too
                                while (m_connection.poll()) {
                                    getMessages();
                                }
                            } catch (Exception ex) {
                                MessageSocket.this.disconnect();
                            }

                            // wake up the notification thread
                            synchronized (this) {
                                //System.out.println("waking up notificaion thread");
                                this.notify();
                            }

                        }

                        // wake the thread up so it dies
                        synchronized (this) {
                            //System.out.println("waking up notificaion thread");
                            this.notify();
                        }
                        
                        System.out.println("receieve thread ended");
                    }
                };
                receieve.setDaemon(true);
                //receieve.setPriority(Thread.MAX_PRIORITY);
                receieve.start();

                Thread notifications = new Thread() {
                    public void run() {
                        System.out.println("notification thread started");

                        while (connected) {
                            while (!messages.isEmpty()) {

                                //System.out.println("running notifications");

                                byte[] message = null;
                                if (!messages.isEmpty()) message = messages.remove(0);

                                if (message != null) {
                                    MessageSocket.this.notifyObservers(message);
                                }
                            }
                            try {
                                synchronized (receieve) {
                                        //System.out.println("notification waiting for messages");
                                        receieve.wait();
                                }
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }

                        System.out.println("notification thread ended");
                    }
                };
                notifications.setDaemon(true);
                notifications.start();

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

    public static boolean getDebug() {
        return debug;
    }

    public static void setDebug(boolean state) {
        debug = state;
    }

    public void setEnabled(boolean state) {
        this.enabled = state;
    }

    public void disconnect() {
        if (!this.connected) return;

        this.connected = false;

        if (!debug) {
            try {
                if (m_connection != null) m_connection.disconnect();
            } catch (SpreadException ex) { }
        }

        m_connection = null;

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

    public void addMembershipObserver(MembershipObserver obs) {
        this.m_membership_observers.add(obs);
    }

    public void removeMembershipObserver(MembershipObserver obs) {
        this.m_membership_observers.remove(obs);
    }

    public void sendMessage(Message m) throws IOException {

        System.out.println("Sending " + m);

        if (debug) return;

        if (this.enabled) {
            SpreadMessage spreadMessage = new SpreadMessage();
            spreadMessage.setData(m.toBytes());
            spreadMessage.addGroup(m.group());
            spreadMessage.setReliable();

            try {
                m_connection.multicast(spreadMessage);
            } catch (SpreadException ex) {
                ex.printStackTrace();
                throw new IOException(ex.getMessage());
            }
        }
    }
}
