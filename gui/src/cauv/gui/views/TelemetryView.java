package cauv.gui.views;

import java.awt.Color;
import java.io.IOException;
import java.util.Vector;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.xy.DefaultTableXYDataset;
import org.jfree.data.xy.XYSeries;

import cauv.auv.AUV;
import cauv.gui.ScreenView;

import com.trolltech.qt.core.QTimer;
import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class TelemetryView extends QWidget implements ScreenView {

    protected AUV auv;
    
	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_TelemetryView ui = new Ui_TelemetryView();

    Vector<Float> depths = new Vector<Float>();
    Vector<Float> pressureFore = new Vector<Float>();
    Vector<Float> pressureAft = new Vector<Float>();
    
    public TelemetryView() {
        ui.setupUi(this);
        ui.graphScroll.setBackgroundRole(ColorRole.NoRole);
    
        QTimer t = new QTimer(this);
        t.timeout.connect(this, "updateGraphs()");
        t.setSingleShot(false);
        t.setInterval(500);
        t.start();
        
    }

    protected QPixmap plotGraph(DefaultTableXYDataset dataset, int width, int height){

        JFreeChart chart = ChartFactory.createTimeSeriesChart(
                null, null, null, dataset, true, false, false);
        formatChart(chart);
        // hide the axes in the icon version
        chart.getXYPlot().getDomainAxis().setVisible(false);
        chart.getXYPlot().getRangeAxis().setVisible(false);
        return chartToPixmap(chart, width, height);
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
        this.auv = auv;
        
        auv.depthChanged.connect(this, "onDepthData(float)");
    }
    
    public void onPressureData(float fore, float aft){
        pressureFore.add(fore);
        if(pressureFore.size() > 500)
            pressureFore.remove(0);
        
        pressureAft.add(fore);
        if(pressureAft.size() > 500)
            pressureAft.remove(0);
        
    }
    
    public void onDepthData(float depth){
        depths.add(depth);
        if(depths.size() > 500)
            depths.remove(0);
    }
    
    public void updateGraphs(){

        XYSeries pressureSeriesFore = new XYSeries("Pressure - Fore", true, false);
        XYSeries pressureSeriesAft = new XYSeries("Pressure - Aft", true, false);
        XYSeries depthSeries = new XYSeries("Depth", true, false);
        for(float depth : depths)
            depthSeries.add(depthSeries.getItemCount()+1, depth);
        for(float p : pressureAft)
            pressureSeriesFore.add(pressureSeriesFore.getItemCount()+1, p);
        for(float p : pressureFore)
            pressureSeriesFore.add(pressureSeriesFore.getItemCount()+1, p);
        DefaultTableXYDataset xyDataset= new DefaultTableXYDataset();
        xyDataset.addSeries(depthSeries);
        xyDataset.addSeries(pressureSeriesFore);
        xyDataset.addSeries(pressureSeriesAft);
        icon.setPixmap(plotIconGraph(xyDataset, 88, 70));
        ui.depthGraph.setPixmap(plotGraph(xyDataset, ui.depthGraph.width(), ui.depthGraph.height()));
    }
    
    @Override
    public void onDisconnect(AUV auv) {
       auv = null; 
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
