
package cauv;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;


/**
 *
 * @author Andy Pritchard
 */
public class Config {

    static transient private final String CONFIG_FILE = "config.cauv";
    private static Properties configFile = new Properties();


    /* config properties */
    public static int GAMEPAD_ID = 2;
    public static int AUV_PORT = 22345;

    public static String ADDRESS = "10.0.0.2";
    public static String LAST_PATH = " ";

    

    public static void load() throws IOException
    {
        FileInputStream fis = new FileInputStream(CONFIG_FILE);
        configFile.load(fis);

        GAMEPAD_ID = Integer.parseInt(configFile.getProperty("GAMEPAD_ID"));
        AUV_PORT = Integer.parseInt(configFile.getProperty("AUV_PORT"));

        ADDRESS = configFile.getProperty("ADDRESS");
        LAST_PATH = configFile.getProperty("LAST_PATH");
        
        fis.close();
    }
    
    
    public static void save() throws FileNotFoundException, IOException
    {
        configFile.setProperty("GAMEPAD_ID", Integer.toString(GAMEPAD_ID));
        configFile.setProperty("AUV_PORT", Integer.toString(AUV_PORT));

        configFile.setProperty("ADDRESS", ADDRESS);
        configFile.setProperty("LAST_PATH", LAST_PATH);
        
        FileOutputStream fos = new FileOutputStream(CONFIG_FILE);
        configFile.store(fos, ADDRESS);
    }
    
}
