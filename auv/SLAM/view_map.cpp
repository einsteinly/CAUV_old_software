#include <vector>
#include <map>
#include <sstream>

#include <QScrollBar>
#include <QMouseEvent>
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsItem>

#include <boost/math/constants/constants.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <utility/options.h>

#include <cauv_slam/SLAMMap.h>

#include "ros_save.hpp"
#include "sonar_ops.h"

using namespace cauv;

class MapView : public QGraphicsView {
    public:
    MapView(QWidget *parent = NULL);
    protected:
    virtual void wheelEvent(QWheelEvent *event);

    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    private:
    bool dragging;
    QMouseEvent lastMouseEvent;
};

MapView::MapView(QWidget *parent) :
    QGraphicsView(parent),
    dragging(false),
    lastMouseEvent(QEvent::None, QPoint(), Qt::NoButton, 0, 0)
{
}

void MapView::mousePressEvent(QMouseEvent *event) {
    QGraphicsView::mousePressEvent(event);
    if (event->button() == Qt::MiddleButton) {
        event->accept();
        dragging = true;
        viewport()->setCursor(Qt::ClosedHandCursor);
    }
}

void MapView::mouseMoveEvent(QMouseEvent *event) {
    QGraphicsView::mouseMoveEvent(event);
    if (dragging) {
        QScrollBar *hBar = horizontalScrollBar();
        QScrollBar *vBar = verticalScrollBar();
        QPoint delta = event->pos() - lastMouseEvent.pos();
        hBar->setValue(hBar->value() + (isRightToLeft() ? delta.x() : -delta.x()));
        vBar->setValue(vBar->value() - delta.y());
    }
    lastMouseEvent = QMouseEvent(QEvent::MouseMove, event->pos(), event->globalPos(),
                                 event->button(), event->buttons(), event->modifiers());
}

void MapView::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);

    if (dragging && event->button() == Qt::MiddleButton) {
        viewport()->setCursor(Qt::OpenHandCursor);
        dragging = false;
    }
}

void MapView::wheelEvent(QWheelEvent *event) {
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    static const double scaleFactor = 1.1;
    if (event->delta() > 0) {
        scale(scaleFactor, scaleFactor);
    } else {
        scale(1 / scaleFactor, 1 / scaleFactor);
    }
}

void MapView::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Q) {
        close();
    }
}

struct Pose {
    float x;
    float y;
    float bearing;
    unsigned int scan_id;
};

class PoseItem : public QGraphicsItemGroup {
    public:
    PoseItem(Pose pose, PoseItem *prev);
    private:
    Pose pose;
    PoseItem *prev;
};

PoseItem::PoseItem(Pose pose_, PoseItem *prev_) :
    pose(pose_), prev(prev_) {
    setPos(QPointF(pose.x, pose.y));
    auto mark = new QGraphicsEllipseItem(0,0,0.3,0.3);
    mark->setBrush(prev ? Qt::white : Qt::green);
    addToGroup(mark);
    mark->setPos(-0.15,-0.15);
    if (prev) {
        auto line = new QGraphicsLineItem(pose.x, pose.y, prev->pose.x, prev->pose.y);
        line->setPen(QPen(Qt::white));
        addToGroup(line);
        auto old_line = new QGraphicsLineItem(pose.x, pose.y, 
                                              pose.x + std::cos(prev->pose.bearing) * 3,
                                              pose.y + std::sin(prev->pose.bearing) * 3);
        old_line->setPen(QPen(Qt::red));
        addToGroup(old_line);
    }
    auto line = new QGraphicsLineItem(pose.x, pose.y, 
                                      pose.x + std::cos(pose.bearing) * 3,
                                      pose.y + std::sin(pose.bearing) * 3);
    line->setPen(QPen(Qt::green));
    addToGroup(line);

}

QImage sonar_mat_to_qimage(cv::Mat matrix) {
    QImage image(matrix.cols, matrix.rows, QImage::Format_ARGB32_Premultiplied);
    uint8_t *image_bytes = image.bits();
    for (int ii = 0; ii < matrix.rows; ii++) {
        for (int jj = 0; jj < matrix.cols; jj++) {
            uint8_t pixel = matrix.at<uint8_t>(ii, jj);
            if (!pixel) {
                static const uint8_t empty_val = 0;
                image_bytes[(ii * matrix.cols + jj) * 4 + 0] = empty_val; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 1] = empty_val; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 2] = empty_val; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 3] = empty_val; 
            } else {
                image_bytes[(ii * matrix.cols + jj) * 4 + 0] = pixel; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 1] = pixel; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 2] = pixel; 
                image_bytes[(ii * matrix.cols + jj) * 4 + 3] = 255; 
            }
        }
    }
    return image;
}

class QImageTiling {
    public:
    QImageTiling(int side_len);
    std::vector<std::pair<QImage*,std::pair<int,int>>> getInBounds(QRectF bounds);

    std::map<std::pair<int,int>, QImage*> image_map;
    private:
    const int side_len;
};

QImageTiling::QImageTiling(int side_len_) :
    side_len(side_len_) {

}

std::vector<std::pair<QImage*,std::pair<int,int>>>
QImageTiling::getInBounds(QRectF bounds) {
    std::vector<std::pair<QImage*,std::pair<int,int>>> images;
    int left   = std::floor(bounds.left()   / side_len) * side_len;
    int right  = std::ceil (bounds.right()  / side_len) * side_len;
    int top    = std::floor(bounds.top()    / side_len) * side_len;
    int bottom = std::ceil (bounds.bottom() / side_len) * side_len;
    for (int x = left; x < right; x += side_len) {
        for (int y = top; y < bottom; y += side_len) {
            auto coords = std::make_pair(x,y);
            QImage *image = image_map[coords];
            if (!image) {
                image = new QImage(side_len, side_len, QImage::Format_ARGB32_Premultiplied);
                image->fill(0);
                image_map[coords] = image;
            }
            images.push_back(std::make_pair(image,coords));
        }
    }
    return images;
}

class SonarBitmap {
    public:
    SonarBitmap(QGraphicsScene *scene, std::vector<boost::filesystem::path> image_paths);
    void addPose(Pose &p);
    void showTiles();
    private:
    QGraphicsScene *scene;
    std::vector<boost::filesystem::path> image_paths;
    QImageTiling tiling;
};

SonarBitmap::SonarBitmap(QGraphicsScene *scene_, std::vector<boost::filesystem::path> image_paths_) :
    scene(scene_), image_paths(image_paths_), tiling(100) {

}

void SonarBitmap::addPose(Pose &p) {
    static const float pi = boost::math::constants::pi<float>();
    auto file = image_paths.at(p.scan_id);
    cauv_slam::SonarImage image;
    load_sonar_image(file, image);
    std::cout << "Range: " << image.rangeEnd << "(" << image.rangeConversion << "m per bin)" << std::endl;
    auto mapping = get_polar_mapping(image);
    auto map_mat = get_polar_to_cartesian_map(mapping, 10);
    auto mat = sonar_msg_to_mat(image);
    cv::Mat cartesian_mat(map_mat.rows, map_mat.cols, CV_8UC1);
    cv::remap(mat, cartesian_mat, map_mat, cv::Mat(), CV_INTER_LINEAR);

    auto pixmap = QPixmap::fromImage(sonar_mat_to_qimage(cartesian_mat));

    QTransform transform;
    transform.translate(p.x, p.y);
    transform.rotateRadians(p.bearing - pi/2);
    transform.translate(-pixmap.width() / 2.0, 0);

    QRectF bounds = transform.mapRect(QRectF(0,0,pixmap.width(),pixmap.height()));

    //scene->addRect(bounds,QPen(Qt::white));

    //This would be a complete mess without C++11
    for (auto image_tile: tiling.getInBounds(bounds)) {
        auto image = image_tile.first;
        int x = image_tile.second.first;
        int y = image_tile.second.second;
        QPainter paint(image);
        paint.setOpacity(0.4);
        paint.translate(-x,-y);
        paint.setWorldTransform(transform, true);
        paint.drawPixmap(0,0, pixmap);
    }
}

void SonarBitmap::showTiles() {
    for (auto image_tile: tiling.image_map) {
        auto image = image_tile.second;
        int x = image_tile.first.first;
        int y = image_tile.first.second;
        //scene->addRect(x,y,image->width(), image->height(), QPen(Qt::green));
        auto item = scene->addPixmap(QPixmap::fromImage(*image));
        item->setPos(x,y);
        item->setZValue(-1);
    }
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    cauv::Options options("View a SLAM map");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("map,m", po::value<std::string>()->required(), "SLAM map to view")
        ("directory,d", po::value<std::string>(), "Directory of sonar images to load")
        ("every,e", po::value<int>()->default_value(10), "How frequently to add scan images to map")
        ("n_scans,n", po::value<int>()->default_value(9999), "How many scans to show on map")
      ;
    if (options.parseOptions(argc, argv)) {
        return 0;
    }

    auto vm = options.vm;
    cauv_slam::SLAMMap map;
    load_message(vm["map"].as<std::string>(), map);

    auto scene = new QGraphicsScene();
    scene->setBackgroundBrush(Qt::black);
    PoseItem *last_pose = nullptr;
    Pose current_pose {0,0,0,0};
    SonarBitmap bitmap(scene, get_msg_files(vm["directory"].as<std::string>())); 
    int i = 0;
    int every = vm["every"].as<int>();
    int n_scans = vm["n_scans"].as<int>();
    for (auto &ipose : map.poses) {
        //urg, minus signs....
        current_pose.x -= ipose.dx * 10;
        current_pose.y -= ipose.dy * 10;
        current_pose.bearing -= ipose.dtheta;
        current_pose.scan_id = ipose.scan_id;
        auto poseitem = new PoseItem(current_pose, last_pose);
        scene->addItem(poseitem);
        std::cout << current_pose.x << " " << current_pose.y << std::endl;
        last_pose = poseitem;
        i++;
        if (!(i % every)) {
            bitmap.addPose(current_pose);
        }
        if (i > n_scans) {
            break;
        }
    }
    bitmap.showTiles();
    auto widget = new MapView();
    widget->setScene(scene);
    widget->show();
    app.exec();

}
