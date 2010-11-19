
package cauv.gui.views;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 *
 * @author Andy Pritchard
 */
public class MissionFile {

    File missionFile;

    public MissionFile(File file) {
        missionFile = file;
    }

    public void writeFile(String c) throws FileNotFoundException, IOException {
        FileOutputStream os = null;
        os = new FileOutputStream(missionFile);
        os.write(c.getBytes(), 0, c.getBytes().length);
    }

    public String readFile() throws FileNotFoundException, IOException {
        String content = "";
        BufferedInputStream bi =
                new BufferedInputStream(new FileInputStream(missionFile));

        while (bi.available() != 0) {
            content += (char) bi.read();
        }

        bi.close();

        return content;
    }

    @Override
    public String toString() {
        return missionFile.getAbsolutePath();
    }

    public String filename()
    {
        return missionFile.getName();
    }

    public boolean exists()
    {
        return missionFile.exists();
    }
}
