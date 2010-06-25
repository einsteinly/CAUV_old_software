package cauv.gui.views;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;

import cauv.Config;
import cauv.auv.AUV;
import cauv.gui.ScreenView;

import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QMessageBox.StandardButton;
import com.trolltech.qt.gui.QMessageBox.StandardButtons;

public class MissionControlView extends QWidget implements ScreenView {

	public Signal1<String> missionRunRequest = new Signal1<String>();
	
	private static class Session {
        // an editor session consists of a file (the model data model)
        // and the editor area (the view and control for the model)

        public QTextEdit area;
        public MissionFile file;
        public boolean modified = false;

        public Session(String filename) {
            file = new MissionFile(new File(filename));
            
            area = new QTextEdit();
            area.textChanged.connect(this, "onModification()");
            area.setDocumentTitle(file.filename());
            QFont font = new QFont();
            font.setFamily("Courier New");
            font.setPointSize(11);
            area.setFont(font);
            new LuaSyntaxHighlighter(area.document());
            
            // tell the manager we've just been created
            SessionManager.registerSession(this);
        }

        public void save() throws FileNotFoundException, IOException {
            file.writeFile(area.document().toPlainText());
            modified = false;
        }

        public void saveAs(File f) throws FileNotFoundException, IOException {
            // first kill the old session by this name
            SessionManager.killSession(this);

            // replace the old MissionFile
            // and then update the sessions name
            file = new MissionFile(f);
            area.setDocumentTitle(file.filename());

            // tell the manager about the new session name
            SessionManager.registerSession(this);

            // and finally do the save
            save();
        }

        public void load() throws FileNotFoundException, IOException {
            area.setText(file.readFile());
            this.modified = false;
        }

        public boolean fileExists() {
            return file.exists();
        }

        @SuppressWarnings("unused")
        private void onModification(){
        	modified = true;
        }
        
        public boolean modified() {
            return modified;
        }
    }

    private static class SessionManager {

        // we need to store both a mapping from filename to Session
        // so we don't open more than one editor tab per file
        // and also a mapping from editor tab to Session so we
        // know which file the editor contains
        private static HashMap<String, Session> fileMap =
                new HashMap<String, Session>();
        private static HashMap<QTextEdit, Session> editorMap =
                new HashMap<QTextEdit, Session>();

        public static Session getSession(MissionFile file) {
            return fileMap.get(file.toString());
        }

        public static Session getSession(QTextEdit area) {
            return editorMap.get(area);
        }

        public static boolean sessionExists(MissionFile file) {
            return fileMap.containsKey(file.toString());
        }

        public static boolean sessionExists(QTextEdit area) {
            return editorMap.containsKey(area);
        }

        public static void registerSession(Session s) {
            editorMap.put(s.area, s);
            fileMap.put(s.file.toString(), s);
        }

        public static void killSession(Session s) {
            editorMap.remove(s.area);
            fileMap.remove(s.file.toString());
        }
    }

	
	
    Ui_MissionControlView ui = new Ui_MissionControlView();
    QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    private int newFileCounter;
    
    public MissionControlView() {
        ui.setupUi(this);
		icon.setPixmap(new QPixmap("classpath:cauv/gui/resources/missioncontrol.png"));
		
		ui.newFile.clicked.connect(this, "newFile()");
        ui.openFile.clicked.connect(this, "openFile()");
        ui.saveButton.clicked.connect(this, "saveMissionFile()");
        ui.saveAsButton.clicked.connect(this, "saveAsMissionFile()");
		ui.openFiles.tabCloseRequested.connect(this, "closeFile(int)");
    }

    
    private void addSessionToEditor(Session s){
    	QIcon icon = new QIcon(new QPixmap("classpath:cauv/gui/resources/luadocument.png"));
    	ui.openFiles.addTab(s.area, icon, s.file.filename());
        ui.openFiles.setCurrentWidget(s.area);
    }
    
    
    public void newFile(){
    	addSessionToEditor(new Session("new file "+ newFileCounter++ +".lua" ));
    }
    
    public void closeFile(int id){
    	if(ui.openFiles.widget(id) instanceof QTextEdit){
	    	Session s = SessionManager.getSession((QTextEdit)ui.openFiles.widget(id));
	    	if(s.modified())
	    	{
	    		StandardButtons buttons = new QMessageBox.StandardButtons(StandardButton.Yes,
	    				StandardButton.No, StandardButton.Cancel);
	    		
	    		StandardButton b = QMessageBox.question(this, "File Modification",
	    				"File has been modifed since last save. Would you like to save the file?",
	    				buttons);
	    		
	    		if(b == StandardButton.Yes)
	    		{
	    			System.err.println("Saved");
	    		} else if (b == StandardButton.Cancel){
	    			return;
	    		}
	    	}
	    	
	    	SessionManager.killSession(s);
	    	ui.openFiles.removeTab(id);
    	}
    }
    
    public void openFile() {
        QFileDialog files = new QFileDialog(this, "Select a file");
        try {

            if (Config.LAST_PATH != null) {
                files.setDirectory(Config.LAST_PATH);
            }

            files.setFileMode(QFileDialog.FileMode.ExistingFile);
            files.show();

            if (files.exec() == 0) return;

            String file = files.selectedFiles().get(0);
            Config.LAST_PATH = files.directory().absolutePath();

            // if a session doesn't exist for this file already then we need to
            // create one and add it to the tabs
            // otherwise we can reuse the session to avoid having the file open
            // in multiple tabs
            MissionFile f = new MissionFile(new File(file));
            Session session = SessionManager.getSession(f);
            if (session == null) {
                session = new Session(file);
                session.load();
                addSessionToEditor(session);
            } else {
                // if the user tried to open a file has been modified in a tab
                // already then we need to tell them about this
                ui.openFiles.setCurrentWidget(session.area);
                if (session.modified()) {
                    StandardButtons buttons = new QMessageBox.StandardButtons(StandardButton.Yes,
                            StandardButton.No);

                    StandardButton b = QMessageBox.question(this, "File Modification",
                            "File has been modifed since last save. Reload file?", buttons);

                    if (b == StandardButton.Yes) session.load();
                }
            }
        } catch (Exception ex) {
            System.err.println(ex.getMessage());
        }

    }

    public void saveMissionFile() {
        try {
            // if we don't have a session open and focused then there's nothing
            // to do, so return
            if (!(ui.openFiles.currentWidget() instanceof QTextEdit)) {
                return;
            }

            // if a session exists for this file and the session hasn't been saved
            // or loaded from a file already then the user has to pick a name
            // for the file
            Session session = SessionManager.getSession((QTextEdit) ui.openFiles.currentWidget());
            if (session == null || !session.fileExists()) {
                saveAsMissionFile();
                return;
            }

            // if a file already exists and the content is different to the editor
            // then check they really want to save over this file.
            StandardButtons buttons = new QMessageBox.StandardButtons(StandardButton.Yes,
                    StandardButton.No);

            StandardButton b = QMessageBox.question(this, "File Modification",
                    "A file with this name already exists. Would you like to overwrite the file?", buttons);

            if (b == StandardButton.Yes){
                session.save();
                return;
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public void saveAsMissionFile() {
        try {
            if (!(ui.openFiles.currentWidget() instanceof QTextEdit)) {
                return;
            }

            QFileDialog files = new QFileDialog(this, "Select a file");
            if (Config.LAST_PATH != null) 
                files.setDirectory(Config.LAST_PATH);

            files.setFileMode(QFileDialog.FileMode.AnyFile);
            files.show();

            if (files.exec() == 0) return;

            String file = files.selectedFiles().get(0);
            Config.LAST_PATH = files.directory().absolutePath();
 

            // get the session the user is working with and save it as the
            // file selected above
            Session session = SessionManager.getSession((QTextEdit) ui.openFiles.currentWidget());
            session.saveAs(new File(file));
            // make sure we rename the tab for consistency
            ui.openFiles.setTabText(ui.openFiles.currentIndex(), session.file.filename());
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
    
    public void onConnect(AUV auv){
    	
    }
    
    @Override
    public void onDisconnect(AUV auv) {
        // TODO Auto-generated method stub
        
    }
    
	@Override
	public QGraphicsItemInterface getIconWidget() {
		return icon;
	}

	@Override
	public String getScreenName() {
		return "Mission Control";
	}

	@Override
	public QWidget getScreenWidget() {
		return this;
	}
}
