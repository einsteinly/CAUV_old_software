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

static const float MAP_SCALE = 10;

void save_sim_scans(std::string env_file,
                    std::string output_dir,
                    std::vector<Pose> poses) {
    std::cout << "Saving scans" << std::endl;
    static const float pi = boost::math::constants::pi<float>();

    auto env_mat = cv::imread(env_file, CV_LOAD_IMAGE_GRAYSCALE);
    auto mapping = PolarMapping::generate();
    int range_bins = std::floor(0.5 + (mapping.rangeEnd - mapping.rangeStart) / mapping.rangeConversion);
    int bearing_bins = mapping.bearings.size() - 1;
    for (auto pose: poses) {
        cv::Mat map(range_bins, bearing_bins, CV_32FC2);
        for (int ii = 0; ii < bearing_bins; ii++) {
            int int_bearing = mapping.bearings.at(ii);
            float actual_bearing = int_bearing / (6400.0 * 0x10000) * 2.0 * pi;
            actual_bearing += pose.bearing;
            float mx = std::cos(actual_bearing);
            float my = std::sin(actual_bearing);
            for (int jj = 0; jj < range_bins; jj++) {
                float x = (jj * mapping.rangeConversion + mapping.rangeStart) * mx + pose.x;
                float y = (jj * mapping.rangeConversion + mapping.rangeStart) * my + pose.y;
                x *= MAP_SCALE;
                y *= MAP_SCALE;
                map.at<cv::Vec2f>(jj, ii) = cv::Vec2f(x,y);
            }
        }
        cv::Mat scan(range_bins, bearing_bins, CV_8UC1);
        cv::remap(env_mat, scan, map, cv::Mat(), CV_INTER_LINEAR);
        cauv_slam::SonarImage scan_msg;
        scan_msg.image.resize(range_bins * bearing_bins);
        for (int ii = 0; ii < bearing_bins; ii++) {
            for (int jj = 0; jj < range_bins; jj++) {
                scan_msg.image[jj * bearing_bins + ii] = scan.at<uint8_t>(jj,ii);
            }
        }
        scan_msg.rangeStart = mapping.rangeStart;
        scan_msg.rangeEnd = mapping.rangeEnd;
        scan_msg.bearings = mapping.bearings;
        scan_msg.rangeConversion = mapping.rangeConversion;
        std::stringstream output_file_name;
        output_file_name << output_dir << "/" << "SonarImage" << 
            std::setfill('0') << std::setw(6) << pose.scan_id << ".rosmsg";
        std::cout << "Saving " << output_file_name.str() << std::endl;
        save_message(output_file_name.str(), scan_msg);
    }
}

void save_sim_map(std::string map_file,
                  std::vector<Pose> poses) {
    std::cout << "Saving map" << std::endl;
    cauv_slam::SLAMMap map;
    Pose last_pose {0,0,0,0};
    for (auto pose: poses) {
        cauv_slam::IncrementalPose i_pose;
        i_pose.dx = pose.x - last_pose.x;
        i_pose.dy = pose.y - last_pose.y;
        i_pose.dtheta = pose.bearing - last_pose.bearing;
        i_pose.scan_id = pose.scan_id;
        std::cout << i_pose << std::endl;
        map.poses.push_back(i_pose);
        last_pose = pose;
    }
    save_message(map_file, map);
}

class PathEdit : public QGraphicsView {
    public:
    PathEdit(std::string map_file,
             std::string img_output_dir,
             std::string env_file,
             QWidget *parent = NULL);
    protected:
    virtual void wheelEvent(QWheelEvent *event);

    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void timerEvent(QTimerEvent *event);
    private:
    int timer_id;
    std::vector<Pose> poses;
    unsigned int pose_index;
    bool dragging;
    QMouseEvent lastMouseEvent;
    std::string map_file;
    std::string img_output_dir;
    std::string env_file;
};

PathEdit::PathEdit(std::string map_file_,
                   std::string img_output_dir_,
                   std::string env_file_, QWidget *parent) :
    QGraphicsView(parent),
    timer_id(0),
    pose_index(0),
    dragging(false),
    lastMouseEvent(QEvent::None, QPoint(), Qt::NoButton, 0, 0),
    map_file(map_file_),
    img_output_dir(img_output_dir_),
    env_file(env_file_)
{
    timer_id = startTimer(50);
}

void PathEdit::mousePressEvent(QMouseEvent *event) {
    QGraphicsView::mousePressEvent(event);
    if (event->button() == Qt::MiddleButton) {
        event->accept();
        dragging = true;
        viewport()->setCursor(Qt::ClosedHandCursor);
    }
}

void PathEdit::mouseMoveEvent(QMouseEvent *event) {
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

void PathEdit::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);

    if (dragging && event->buttons() == Qt::MiddleButton) {
        viewport()->setCursor(Qt::OpenHandCursor);
        dragging = false;
    }
    lastMouseEvent = QMouseEvent(QEvent::MouseButtonRelease, event->pos(), event->globalPos(),
                                 event->button(), event->buttons(), event->modifiers());
}

void PathEdit::wheelEvent(QWheelEvent *event) {
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    static const double scaleFactor = 1.1;
    if (event->delta() > 0) {
        scale(scaleFactor, scaleFactor);
    } else {
        scale(1 / scaleFactor, 1 / scaleFactor);
    }
}

void PathEdit::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Q) {
        close();
    }
    if (event->key() == Qt::Key_S) {
        for (auto &pose: poses) {
            pose.x /= MAP_SCALE;
            pose.y /= MAP_SCALE;
        }
        save_sim_map(map_file, poses);
        save_sim_scans(env_file, img_output_dir, poses);
    }
}

void PathEdit::timerEvent(QTimerEvent *event) {
    if (lastMouseEvent.type() == QEvent::MouseButtonRelease) {
        return;
    }
    if (lastMouseEvent.buttons() == Qt::LeftButton) {
        auto pos = mapToScene(lastMouseEvent.pos());
        poses.push_back(Pose{static_cast<float>(pos.x()), 
                             static_cast<float>(pos.y()),
                             0,
                             static_cast<unsigned int>(poses.size())});
        std::cout << pos.x() << " " << pos.y() << std::endl;
        scene()->addEllipse(pos.x() - 2.5, pos.y() - 2.5, 5, 5)->setBrush(Qt::white);
    }
    if (lastMouseEvent.buttons() == Qt::RightButton) {
        if (pose_index >= poses.size()) {
            return;
        }
        static const float pi = boost::math::constants::pi<float>();
        auto pos = mapToScene(lastMouseEvent.pos());
        auto& pose = poses[pose_index++];
        auto pose_pos = QPointF(pose.x, pose.y);
        auto diff = pose_pos - pos;
        pose.bearing = std::atan2(diff.x(), diff.y()) + pi / 2;
        std::cout << pose.bearing << std::endl;
        scene()->addLine(pos.x(), pos.y(), pose_pos.x(), pose_pos.y(), QPen(Qt::red));
    }
}


int main(int argc, char **argv) {
    QApplication app(argc, argv);
    cauv::Options options("Create a simulated set of sonar images");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("directory,d", po::value<std::string>()->required(), "Directory to output sonar images to")
        ("environment,e", po::value<std::string>()->required(), "Environment image to use")
        ("map,m", po::value<std::string>()->required(), "SLAM map to output")
      ;
    if (options.parseOptions(argc, argv)) {
        return 0;
    }
    auto vm = options.vm;

    auto scene = new QGraphicsScene();
    auto view = new PathEdit(vm["map"].as<std::string>(),
                             vm["directory"].as<std::string>(),
                             vm["environment"].as<std::string>());
    view->setScene(scene);
    auto display_image = QPixmap::fromImage(QImage(vm["environment"].as<std::string>().c_str()));
    scene->addPixmap(display_image);

    view->show();
    app.exec();

    return 0;
}
