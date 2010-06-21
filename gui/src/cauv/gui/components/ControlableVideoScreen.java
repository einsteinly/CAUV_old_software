package cauv.gui.components;

import cauv.types.floatYPR;

import com.trolltech.qt.core.QPoint;
import com.trolltech.qt.core.Qt.Key;
import com.trolltech.qt.gui.*;

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
	public Signal0 upClick = new Signal0();
	public Signal0 downClick = new Signal0();
	public Signal0 leftClick = new Signal0();
	public Signal0 rightClick = new Signal0();

	public ControlableVideoScreen(QWidget parent) {
		super(parent);
		ui.setupUi(this);
		ui.up.clicked.connect(upClick, "emit()");
		ui.down.clicked.connect(downClick, "emit()");
		ui.left.clicked.connect(leftClick, "emit()");
		ui.right.clicked.connect(rightClick, "emit()");
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
		p.setBrush(QColor.black);

		floatYPR min = pointToAttitude(0, 0);
		
		// y
		if((this.elements & SHOW_Y_AXIS) != 0) {
			p.drawLine(this.width() / 2, 50, this.width() / 2, this.height()-50);
	
			for(int i = (int)min.pitch; i < (int)(min.pitch + axisHeight); i += 10){
				int remainder = (int) (i % 10);
				int y = this.height() - attitudeToPoint(axisWidth/2, i-remainder).y();
				
				if(y > 50 && y < this.height() - 50) {
					p.drawLine((this.width()/2)-5, y,(this.width()/2), y);
		
					if(Math.abs((i-remainder)-pitch)>5)
						p.drawText((this.width()/2) + 5, y+4, i-remainder+"°");
				}
			}
		}
		
		//x
		if((this.elements & SHOW_X_AXIS) != 0) {
			p.drawLine(50, (this.height() / 2), this.width() - 50, this.height() / 2);
			
			for(int i = (int)min.yaw; i < (int)(min.yaw+ axisWidth); i += 10){
				int remainder = (int) (i % 10);
				int x = attitudeToPoint(i-remainder, axisHeight/2).x();
				
				if(x > 50 && x < this.width() - 50) {
					p.drawLine(x, (this.height()/2)-5, x,(this.height()/2));
					
					if(Math.abs((i-remainder)-yaw)>5)
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

	protected void draw(QPainter p) {
		this.paintImage(p);
		this.drawAxes(p);
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
