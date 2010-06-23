package $package

abstract class Message {
    public String group() {
        return m_group;
    }

    public int id() {
        return m_id;
    }

    public abstract byte[] toBytes() throws IOException;

    protected int m_id;
    protected String m_group;

    protected Message(int id, String group) {
        m_id = id;
        m_group = group;
    }

    @Override
    public String toString() {
        return this.getClass().getName() + ": {\n\tID: " + this.id() + "\n\tGroup: " + this.group()
                + "\n}";
    }
}
