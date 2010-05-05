package cauv.gui.components;

import com.trolltech.qt.gui.QPaintEvent;
import com.trolltech.qt.gui.QPainter;
import com.trolltech.qt.gui.QPixmap;
import com.trolltech.qt.gui.QWidget;

public class VideoScreen extends QWidget {
    QPixmap pixmap = new QPixmap();

    public VideoScreen() {
    }
    
    public VideoScreen(QWidget parent) {
        super(parent);
    }

    public void setPixmap(QPixmap pixmap){
    	this.pixmap = pixmap;
    	this.repaint();
    }
    
    protected void paintImage(QPainter p){
    	p.drawPixmap(this.contentsRect(), this.pixmap);
    }
    
    @Override
    protected void paintEvent(QPaintEvent arg) {
    	QPainter p = new QPainter(this);
    	this.paintImage(p);
    	p.end();
    	super.paintEvent(arg);
    }
    
}
