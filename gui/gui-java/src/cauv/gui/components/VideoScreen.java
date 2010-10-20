package cauv.gui.components;

import cauv.types.Image;

import com.trolltech.qt.gui.QBrush;
import com.trolltech.qt.gui.QColor;
import com.trolltech.qt.gui.QImage;
import com.trolltech.qt.gui.QMouseEvent;
import com.trolltech.qt.gui.QPaintEvent;
import com.trolltech.qt.gui.QPainter;
import com.trolltech.qt.gui.QPixmap;
import com.trolltech.qt.gui.QWidget;

public class VideoScreen extends QWidget {
    QPixmap pixmap = new QPixmap();

    Image i;
    
    public Signal1<QMouseEvent> clicked = new Signal1<QMouseEvent>();
    
    
    public VideoScreen() {
    }
    
    public VideoScreen(QWidget parent) {
        super(parent);
    }

    public void setImage(Image img){
        if(img == null) return;
        if(img == i) return;
        i = img;
        
        QImage qImage = new QImage();
        byte [] data = new byte[img.data.size()];
        for(int i = 0; i < img.data.size(); i++)
            data[i] = img.data.get(i);
        qImage.loadFromData(data, img.format);
        this.setPixmap(QPixmap.fromImage(qImage));

    }
    
    public void setPixmap(QPixmap pixmap){
    	this.pixmap = pixmap;
    	this.repaint();
    }
    
    public QPixmap getPixmap() {
        return pixmap;
    }
    
    protected void mouseReleaseEvent(QMouseEvent event){
    	clicked.emit(event);
    }
    
    protected void paintImage(QPainter p){
        p.fillRect(this.rect(), new QBrush(QColor.black));
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
