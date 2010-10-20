package cauv.gui.components;

import java.awt.Label;

import cauv.types.floatYPR;

import com.trolltech.qt.core.QPoint;
import com.trolltech.qt.core.QPointF;
import com.trolltech.qt.core.Qt.BGMode;
import com.trolltech.qt.core.Qt.Key;
import com.trolltech.qt.core.Qt.TextFormat;
import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class ControlableVideoScreen extends VideoScreen {
	

	public static transient int SHOW_NONE = 0;
	public static transient int SHOW_DOT = 1;
	public static transient int SHOW_Y_AXIS = 2;
	public static transient int SHOW_X_AXIS = 4;
	public static transient int SHOW_TEXT = 8;
	
	
	int elements = SHOW_DOT | SHOW_Y_AXIS | SHOW_X_AXIS | SHOW_TEXT;
	
	Ui_ControlableVideoScreen ui = new Ui_ControlableVideoScreen();

	float axisHeight = 120;
	float axisWidth = 120;
	
	float targetPitch = 0;
	float targetYaw = 0;

	float pitch = 0;
	float yaw = 0;
	
	boolean updating = false;

	public Signal2<Float, Float> directionChange = new Signal2<Float, Float>();
	public Signal1<Boolean> upClick = new Signal1<Boolean>();
	public Signal1<Boolean> downClick = new Signal1<Boolean>();
	public Signal1<Boolean> leftClick = new Signal1<Boolean>();
	public Signal1<Boolean> rightClick = new Signal1<Boolean>();

	public ControlableVideoScreen(QWidget parent) {
		super(parent);
		ui.setupUi(this);
        ui.up.pressed.connect(this, "upPress()");
        ui.up.released.connect(this, "upRelease()");

        ui.down.pressed.connect(this, "downPress()");
        ui.down.released.connect(this, "downRelease()");

        ui.left.pressed.connect(this, "leftPress()");
        ui.left.released.connect(this, "leftRelease()");
		
        ui.right.pressed.connect(this, "rightPress()");
        ui.right.released.connect(this, "rightRelease()");
        
	}

    protected void upPress(){
        upClick.emit(true);
    }
    
    protected void upRelease(){
        upClick.emit(false);
    }

    protected void downPress(){
        downClick.emit(true);
    }
    
    protected void downRelease(){
        downClick.emit(false);
    }

    protected void leftPress(){
        leftClick.emit(true);
    }
    
    protected void leftRelease(){
        leftClick.emit(false);
    }

    protected void rightPress(){
        rightClick.emit(true);
    }
    
    protected void rightRelease(){
        rightClick.emit(false);
    }
	
	public void setVisibleElements(int visible){
		this.elements = visible;
	}
	
	public void setXAxisScale(float value){
		this.axisWidth = value;
	}
	
	public void setYAxisScale(float value){
		this.axisHeight = value;
	}
	
	public void setText(String text) {
		this.ui.label.setText(text);
	}

	public void setDotLocation(float yaw, float pitch) {
		this.targetYaw = yaw;
		this.targetPitch = pitch;
		this.repaint();
	}
	
	public void setYaw(float yaw){
	    this.yaw = yaw;
	}
	
	public void setPitch(float pitch) {
        this.pitch = pitch;
    }

	public void setAttitude(floatYPR orientation){
        this.setYaw(orientation.yaw);
        this.setPitch(orientation.pitch);
	}
	
	protected floatYPR pointToAttitude(int x, int y){
		floatYPR ret =  new floatYPR();
		
		float heightRatio = (float)y/(float)this.height();
		float degrees = (axisHeight/2) - (heightRatio * axisHeight);
		ret.pitch = pitch - degrees;
		
		float widthRatio = (float)x/(float)this.width();
		degrees = (axisWidth/2) - (widthRatio * axisWidth);
		ret.yaw = yaw - degrees;
		
		return ret;
	}
	
	protected QPoint attitudeToPoint(float yawInput, float pitchInput){
		QPoint ret =  new QPoint();
		
		float delta = pitch - pitchInput;
		float heightRatio = delta / (float)axisHeight;
		int pixels = (int) (heightRatio * this.height());
		ret.setY(this.height()-(pixels + (this.height()/2)));
		
		delta = yaw - yawInput;
		float widthRatio = delta / (float)axisWidth;
		pixels = (int) (widthRatio * this.width());
		ret.setX(this.width() - (pixels + (this.width()/2)));
		
		return ret;
	}
	
	
	protected void drawAxes(QPainter p) {
		// draw the axes
        p.setBrush(QColor.white);
        p.setPen(QColor.white);

		floatYPR min = pointToAttitude(0, 0);
		
		// y
		if((this.elements & SHOW_Y_AXIS) != 0) {
			p.drawLine(this.width() / 2, 50, this.width() / 2, this.height()-50);
	
			for(int i = (int)min.pitch; i < (int)(min.pitch + axisHeight); i += 5){
				int remainder = (int) (i % 5);
				int y = this.height() - attitudeToPoint(axisWidth/2, i-remainder).y();
				
				if(y > 50 && y < this.height() - 50) {
					p.drawLine((this.width()/2)-5, y,(this.width()/2), y);
		
					if(Math.abs((i-remainder)-pitch)>2.5)
						p.drawText((this.width()/2) + 5, y+4, i-remainder+"°");
				}
			}
		}
		
		//x
		if((this.elements & SHOW_X_AXIS) != 0) {
			p.drawLine(50, (this.height() / 2), this.width() - 50, this.height() / 2);
			
			for(int i = (int)min.yaw; i < (int)(min.yaw+ axisWidth); i += 5){
				int remainder = (int) (i % 5);
				int x = attitudeToPoint(i-remainder, axisHeight/2).x();
				
				if(x > 50 && x < this.width() - 50) {
					p.drawLine(x, (this.height()/2)-5, x,(this.height()/2));
					
					if(Math.abs((i-remainder)-yaw)>2.5)
						p.drawText(x - 8, (this.height()/2)+15, i-remainder+"°");
				}
			}
		}
	}


	@Override
	protected void keyPressEvent(QKeyEvent arg) {
		if (arg.key() == Key.Key_Up.value())
			this.pitch += 0.3;
		if (arg.key() == Key.Key_Down.value())
			this.pitch -= 0.3;
		if (arg.key() == Key.Key_Right.value())
			this.yaw += 0.3;
		if (arg.key() == Key.Key_Left.value())
			this.yaw -= 0.3;
		repaint();
		super.keyPressEvent(arg);
	}
    
    protected void drawDot(QPainter p) {
        if((this.elements & SHOW_DOT) != 0) {
            int rad = 4;
            p.setBrush(QColor.red);
            QPoint point = attitudeToPoint(targetYaw, targetPitch);
            p.drawEllipse(point.x() - rad, this.height() - point.y() - rad, 2 * rad, 2 * rad);
        }
    }
    
    protected void drawText(QPainter p) {
        /*QPainterPath path = new QPainterPath();
        path.addText(new QPointF(50, 60), new QFont("Arial", 11), ui.label.text());
       
        QColor color = new QColor(QColor.white);
        color.setAlphaF(0.3);
        //QBrush brush = new QBrush(color);
        //p.fillPath(path, brush);
        
        p.setPen(new QPen(color, 4));
        p.drawPath(path);
        
        //p.setPen(new QPen(QColor.black, 1));
        //p.drawPath(path);*/
    }

	protected void draw(QPainter p) {
		this.paintImage(p);
		this.drawAxes(p);
        this.drawText(p);
		this.drawDot(p);
	}

	@Override
	protected void paintEvent(QPaintEvent arg) {
		QPainter p = new QPainter();
		p.begin(this);
		this.draw(p);
		p.end();
	}
	
	protected void handleMouseEvent(QMouseEvent arg){
		if((this.elements & SHOW_DOT) != 0) {
			floatYPR attitude = pointToAttitude(arg.x(), this.height()-arg.y());
			float targetYaw = attitude.yaw;
			float targetPitch = attitude.pitch;
			/*
			Main.trace("start y="+(this.height()-arg.y()));
			Main.trace("start x="+(arg.x()));
			float pitch = .pitch;
			Main.trace("pitch="+pitch);
			float yaw = pointToAttitude(arg.x(), this.height()-arg.y()).yaw;
			Main.trace("yaw="+yaw);
			Main.trace("end y="+this.attitudeToPoint(yaw, pitch));*/
			
			this.setDotLocation(targetYaw, targetPitch);
		}
	}

	@Override
	protected void mouseMoveEvent(QMouseEvent arg) {
		if (updating) {
			handleMouseEvent(arg);
			directionChange.emit(targetYaw, targetPitch);
		}
		super.mouseMoveEvent(arg);
	}

	@Override
	protected void mouseReleaseEvent(QMouseEvent arg) {
		if(updating)
			directionChange.emit(targetYaw, targetPitch);
		updating = false;
		super.mouseReleaseEvent(arg);
	}

	@Override
	protected void mousePressEvent(QMouseEvent arg) {
		updating = true;
		handleMouseEvent(arg);
		super.mousePressEvent(arg);
		this.grabKeyboard();
	}
}
