package cauv.gui.views;

import java.awt.Color;
import java.io.IOException;
import java.util.HashMap;
import java.util.Vector;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.xy.DefaultTableXYDataset;
import org.jfree.data.xy.XYSeries;

import cauv.auv.AUV;
import cauv.gui.ScreenView;
import cauv.types.floatYPR;

import com.trolltech.qt.core.QTimer;
import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class TelemetryView extends QWidget implements ScreenView {

    
    class Series<T> {
        int maxPoints = 500;
        String name;
        Vector<T> points = new Vector<T>();
        
        public Series(String name, int maxPoints) {
            this.name = name;
            this.maxPoints = maxPoints;
        }
        
        public synchronized void onPointData(T point){
            points.add(point);
            if(points.size() > maxPoints)
                points.remove(0);  
        }
        
        public Vector<T> getPoints() {
            return points;
        }
        
        public String getName() {
            return name;
        }
    }
    
    
    class GraphView extends QLabel{
        
        Vector<Series<?>> series = new Vector<Series<?>>();
        
        public GraphView() {
            this.setMaximumHeight(150);
        }
        
        public void addSeries(Series<?> series){
            this.series.add(series);
        }
        
        protected void updatePixmap(){
            // setup dataset
            DefaultTableXYDataset dataset= new DefaultTableXYDataset();
            
            
            for(Series<?> s : series){
                
                HashMap<String, XYSeries> namedSeries = new HashMap<String, XYSeries>();
 
                synchronized (s) {
                    for(Object t : s.getPoints()){
                        if(t instanceof Number){
                            if(!namedSeries.containsKey(s.getName()))
                                namedSeries.put(s.getName(), new XYSeries(s.getName(), true, false));
                            
                            XYSeries xySeries = namedSeries.get(s.getName());
                            xySeries.add(xySeries.getItemCount()+1, (Number) t);
                        } else if(t instanceof floatYPR){
                           t = (floatYPR) t;
                            
                            if(!namedSeries.containsKey(s.getName() + " Yaw"))
                                namedSeries.put(s.getName() + " Yaw", new XYSeries(s.getName() + " Yaw", true, false));
                            XYSeries xySeries = namedSeries.get(s.getName() + " Yaw");
                            xySeries.add(xySeries.getItemCount()+1, ((floatYPR) t).yaw);
                            
                            if(!namedSeries.containsKey(s.getName() + " Pitch"))
                                namedSeries.put(s.getName() + " Pitch", new XYSeries(s.getName() + " Pitch", true, false));
                            xySeries = namedSeries.get(s.getName() + " Pitch");
                            xySeries.add(xySeries.getItemCount()+1, ((floatYPR) t).pitch);
                            
                            if(!namedSeries.containsKey(s.getName() + " Roll"))
                                namedSeries.put(s.getName() + " Roll", new XYSeries(s.getName() + " Roll", true, false));
                            xySeries = namedSeries.get(s.getName() + " Roll");
                            xySeries.add(xySeries.getItemCount()+1, ((floatYPR) t).roll);
                        }
                    }
                }
                
                for(XYSeries ser : namedSeries.values())
                    dataset.addSeries(ser);
            }
            
            // chart setup
            JFreeChart chart = ChartFactory.createTimeSeriesChart(
                    null, null, null, dataset, true, false, false);
            
            chart.setBackgroundPaint(Color.BLACK);
            chart.getPlot().setBackgroundAlpha(0);
            chart.setBorderVisible(false);
            
            chart.getXYPlot().getDomainAxis().setVisible(false);
            chart.getXYPlot().getRangeAxis().setVisible(true);
            
            try {
                QPixmap pm = new QPixmap();
                byte [] data =  ChartUtilities.encodeAsPNG(chart.createBufferedImage(this.width(), this.height()));
                pm.loadFromData(data);
                this.setPixmap(pm);
            }
            catch(IOException e) {
                e.printStackTrace();
            }
        }   
    }
    
    
    protected AUV auv;
    
	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_TelemetryView ui = new Ui_TelemetryView();
    
    Series<Float> depth = new Series<Float>("Depth", 500);
    Series<Float> forePressure = new Series<Float>("Pressure Fore", 500);
    Series<Float> aftPressure = new Series<Float>("Pressure Aft", 500);
    Series<floatYPR> orientation = new Series<floatYPR>("Oritentation", 500);
    
    Vector<GraphView> graphs = new Vector<GraphView>();
    
    public TelemetryView() {
        ui.setupUi(this);
        ui.graphScroll.setBackgroundRole(ColorRole.NoRole);

        GraphView pressures = new GraphView();
        pressures.addSeries(depth);
        pressures.addSeries(forePressure);
        pressures.addSeries(aftPressure);
        graphs.add(pressures);
        ui.graphsLayout.addWidget(pressures);
        
        GraphView orientations = new GraphView();
        orientations.addSeries(orientation);
        graphs.add(orientations);
        ui.graphsLayout.addWidget(orientations);
        
        
        QTimer t = new QTimer(this);
        t.timeout.connect(this, "updateGraphs()");
        t.setSingleShot(false);
        t.setInterval(300);
        t.start();
        
    }
    
    public void onConnect(AUV auv){
        this.auv = auv;

        auv.depthChanged.connect(depth, "onPointData(Object)");
        auv.pressureChanged.connect(this, "onPressureData(float, float)");
        auv.orientationChanged.connect(orientation, "onPointData(Object)");
    }
    
    public void onPressureData(float fore, float aft){
        forePressure.onPointData(fore);
        aftPressure.onPointData(aft); 
    }
    
    public void updateGraphs(){

        for(GraphView gv : graphs){
            gv.updatePixmap();
        }
        /*
        
        DefaultTableXYDataset pressureDataset= new DefaultTableXYDataset();
        
        XYSeries depthSeries = new XYSeries("Depth", true, false);
        for(float depth : depths)
            depthSeries.add(depthSeries.getItemCount()+1, depth);
        pressureDataset.addSeries(depthSeries);
        

        XYSeries pressureSeriesFore = new XYSeries("Pressure - Fore", true, false);
        XYSeries pressureSeriesAft = new XYSeries("Pressure - Aft", true, false);
        for(float p : pressureFore)
            pressureSeriesFore.add(pressureSeriesFore.getItemCount()+1, p);
        pressureDataset.addSeries(pressureSeriesFore);
        for(float p : pressureAft)
            pressureSeriesAft.add(pressureSeriesAft.getItemCount()+1, p);
        pressureDataset.addSeries(pressureSeriesAft);
        
        icon.setPixmap(plotIconGraph(pressureDataset, 100, 90));
        ui.depthGraph.setPixmap(plotGraph(pressureDataset, ui.depthGraph.width(), ui.depthGraph.height()));
        
        

        DefaultTableXYDataset orientationDataset= new DefaultTableXYDataset();
        XYSeries yawSeries = new XYSeries("Yaw", true, false);
        XYSeries pitchSeries = new XYSeries("Pitch", true, false);
        for(floatYPR orientation : this.orientation) {
            pitchSeries.add(pitchSeries.getItemCount()+1, orientation.pitch);
            yawSeries.add(yawSeries.getItemCount()+1, orientation.yaw);
        }
        orientationDataset.addSeries(yawSeries);
        orientationDataset.addSeries(pitchSeries);
        
        ui.orientationGraph.setPixmap(plotGraph(orientationDataset, ui.orientationGraph.width(), ui.orientationGraph.height()));        
    */}
    
    @Override
    public void onDisconnect() {
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
