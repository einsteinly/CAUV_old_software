package cauv.gui.views;

import java.awt.Color;
import java.io.IOException;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.DefaultTableXYDataset;
import org.jfree.data.xy.XYSeries;

import cauv.auv.AUV;
import cauv.gui.ScreenView;

import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class TelemetryView extends QWidget implements ScreenView {

	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_TelemetryView ui = new Ui_TelemetryView();

    XYSeries depthSeries = new XYSeries("Depth", true, false);
    
    
    public TelemetryView() {
        ui.setupUi(this);
        ui.graphScroll.setBackgroundRole(ColorRole.NoRole);
        depthSeries.add(depthSeries.getItemCount()+1, depthSeries.getItemCount()+2);
        depthSeries.add(depthSeries.getItemCount()+1, depthSeries.getItemCount()+4);
        depthSeries.add(depthSeries.getItemCount()+1, depthSeries.getItemCount()+8);
        depthSeries.add(depthSeries.getItemCount()+1, depthSeries.getItemCount()+16);

        DefaultTableXYDataset xyDataset= new DefaultTableXYDataset();
        xyDataset.addSeries(depthSeries);
        icon.setPixmap(plotIconGraph(xyDataset, 88, 70));
    }

    protected QPixmap plotIconGraph(DefaultTableXYDataset dataset, int width, int height){

    	JFreeChart chart = ChartFactory.createTimeSeriesChart(
        		null, null, null, dataset, false, false, false);
    	formatChart(chart);
    	// hide the axes in the icon version
        chart.getXYPlot().getDomainAxis().setVisible(false);
        chart.getXYPlot().getRangeAxis().setVisible(false);
        return chartToPixmap(chart, width, height);
    }
    
    protected void formatChart(JFreeChart chart){
        chart.setBackgroundPaint(Color.BLACK);
        chart.getPlot().setBackgroundAlpha(0);
        chart.setBorderVisible(false);
    }
    
    protected QPixmap chartToPixmap(JFreeChart chart, int width, int height){
    	QPixmap pixmap = new QPixmap();
    	try {
        	pixmap.loadFromData(ChartUtilities.encodeAsPNG(chart.createBufferedImage(width, height)));
        }
        catch(IOException e) {}
    	return pixmap;
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
		return "Telemetry";
	}

	@Override
	public QWidget getScreenWidget() {
		return this;
	}
}
