package cauv.gui;

import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class ScreenIcon extends QWidget {

    Ui_ScreenIcon ui = new Ui_ScreenIcon();

    public ScreenIcon() {
        ui.setupUi(this);
    }

    public ScreenIcon(QWidget parent) {
        super(parent);
        ui.setupUi(this);
    }
    
    public void addWidget(QWidget widget){
    	ui.iconLayout.addWidget(widget);
    }
    
    public void setText(String text){
    	ui.iconLabel.setText(text);
    }
    
    public void setSelected(boolean state){
    	if(state){
    		this.setBackgroundRole(ColorRole.Highlight);
    	}
    }
}
