package cauv.auv;

public class MessageTest {

	/**
	 * @param args
	 */
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub

		MessageSocket m = new MessageSocket("192.168.1.6", 4803, "GUITest");
		m.sendMessage(new MotorMessage(MotorID.PROP, (byte)120));
		System.out.print("moo");
	}

}
