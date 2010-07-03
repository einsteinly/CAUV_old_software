package cauv.gui.views;

import java.awt.Color;
import java.awt.Paint;
import java.io.IOException;
import java.util.HashMap;
import java.util.Vector;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisSpace;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.xy.DefaultTableXYDataset;
import org.jfree.data.xy.XYSeries;

import cauv.auv.AUV;
import cauv.auv.AUV.DataStream;
import cauv.auv.AUV.MemorisingDataStream;
import cauv.auv.AUV.MultipleDataStreamEmitter;
import cauv.gui.ScreenView;
import cauv.types.MotorDemand;
import cauv.types.floatYPR;

import com.trolltech.qt.core.QEvent;
import com.trolltech.qt.core.QObject;
import com.trolltech.qt.core.QTimer;
import com.trolltech.qt.core.QEvent.Type;
import com.trolltech.qt.core.Qt.Key;
import com.trolltech.qt.gui.*;
import com.trolltech.qt.gui.QPalette.ColorRole;

public class TelemetryView extends QWidget implements ScreenView {

    class GraphView extends QLabel{
        
        int max, min;
        Vector<MemorisingDataStream<?>> series = new Vector<MemorisingDataStream<?>>();

        public GraphView(MemorisingDataStream<?> stream, String title, int min, int max) {
            series.add(stream);
            this.max = max;
            this.min = min;
            
            this.setMinimumHeight(150);
        }
        
        public GraphView(Vector<MemorisingDataStream<?>> streams, String title, int min, int max) {
            series.addAll(streams);
            this.max = max;
            this.min = min;
            
            this.setMinimumHeight(150);
        }
        
        public GraphView(MultipleDataStreamEmitter emitter, String title, int min, int max) {
            for(String key : emitter.getDataStreams().keySet()){
                DataStream<?> s = emitter.getDataStreams().get(key);
                if(s instanceof MemorisingDataStream<?>)
                    series.add((MemorisingDataStream<?>)s);
            }
            this.max = max;
            this.min = min;
            
            this.setMinimumHeight(150);
        }
        
        protected void updatePixmap(){
            // setup dataset
            DefaultTableXYDataset dataset= new DefaultTableXYDataset();
            
            for(MemorisingDataStream<?> s : series){
                Vector<?> points = s.getHistory();
                XYSeries xySeries = new XYSeries(s.getName(), true, false);
                
                for(Object t : points){
                    if(!(t instanceof Number)) break;
                    xySeries.add(xySeries.getItemCount()+1, (Number) t);
                }

                dataset.addSeries(xySeries);
            }
            
            // chart setup
            JFreeChart chart = ChartFactory.createTimeSeriesChart(
                    null, null, null, dataset, true, false, false);
            
            chart.setBackgroundPaint(Color.BLACK);
            chart.getPlot().setBackgroundAlpha(0);
            chart.setBorderVisible(false);
            
            chart.getXYPlot().getDomainAxis().setVisible(false);
            chart.getXYPlot().getRangeAxis().setVisible(true);
            chart.getXYPlot().getRangeAxis().setTickLabelPaint(Color.white);
            
            
            //chart.getXYPlot().getRangeAxis().setAutoRange(true);
            //chart.getXYPlot().getRangeAxis().setRange(min, max);
            
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
    
    
    Vector<GraphView> graphs = new Vector<GraphView>();
    
    public TelemetryView() {        
        
        ui.setupUi(this);

        QTimer t = new QTimer(this);
        t.timeout.connect(this, "updateGraphs()");
        t.setSingleShot(false);
        t.setInterval(300);
        t.start();
        
        
        ui.listWidget.itemDoubleClicked.connect(this, "addGraph()");
        ui.listWidget.installEventFilter(this);
        
    }
    
    public boolean eventFilter(QObject obj, QEvent event) {
        
        if (obj == ui.listWidget)
        {
            if (event.type() == Type.KeyPress)
            {
                QKeyEvent keyEvent = (QKeyEvent)event;
                if (keyEvent.key() == Key.Key_Return.value())
                {
                    addGraph();
                     return true;
                }
            }
            return super.eventFilter(obj, event);
        }
        return super.eventFilter(obj, event);
    }
    
    public void addGraph(){
        
        Vector<MemorisingDataStream<?>> streams = new Vector<MemorisingDataStream<?>>();
        
        for(QListWidgetItem item : ui.listWidget.selectedItems()) {
            MemorisingDataStream<?> stream = (MemorisingDataStream<?>) item.data(0);
            streams.add(stream);
        }
        
        GraphView graph = new GraphView(streams, "Many Many Graphs", 0, 10);
        graph.setWindowTitle("Many Many Graphs");
        graphs.add(graph);
        ui.graphs.addSubWindow(graph);
        graph.showNormal();
        ui.listWidget.clearSelection();
    }
    
    public void onConnect(AUV auv){
        this.auv = auv;

        
        Vector<DataStream<?>> streams = AUV.getAllStreams();
        
        for(DataStream<?> stream : streams){
            if(stream instanceof MemorisingDataStream<?>) {
                QListWidgetItem item = new QListWidgetItem(stream.getName());
                item.setData(0, stream);
                ui.listWidget.addItem(item);
            }
        }
        
          }
    
    public void updateGraphs(){
        
        for(GraphView gv : graphs){
            try {
                gv.updatePixmap();
            } catch (Exception ex){
                graphs.remove(gv);
                return;
            }
        }

        if(auv != null)
        auv.autopilots.DEPTH.setTarget((float)Math.random()*1000);
    }
    
    @Override
    public void onDisconnect() {
       auv = null; 
       
       ui.listWidget.clear();
       ui.graphs.closeAllSubWindows();
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
