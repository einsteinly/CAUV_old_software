#include <iostream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QLabel>
#include <QWidget>
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>

#include <utility/options.h>
#include <cauv_slam/SonarImage.h>

#include "manual_layout_view.h"
#include "sonar_ops.h"

QImage sonar_mat_to_qimage(cv::Mat matrix) {
    QImage image(matrix.cols, matrix.rows, QImage::Format_ARGB32_Premultiplied);
    uint8_t *image_bytes = image.bits();
    for (int ii = 0; ii < matrix.rows; ii++) {
        for (int jj = 0; jj < matrix.cols; jj++) {
            uint8_t pixel = matrix.at<uint8_t>(ii, jj);
            image_bytes[(ii * matrix.cols + jj) * 4 + 0] = pixel; 
            image_bytes[(ii * matrix.cols + jj) * 4 + 1] = pixel; 
            image_bytes[(ii * matrix.cols + jj) * 4 + 2] = pixel; 
            image_bytes[(ii * matrix.cols + jj) * 4 + 3] = pixel; 
        }
    }
    return image;
}

class SonarScanItem: public QGraphicsPixmapItem {
    public:
    SonarScanItem(boost::filesystem::path source_path, QGraphicsItem *parent = NULL);
    protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    QTransform curr_transform;
    QTransform new_transform;
    QPointF start_point;
    QPointF rotate_center;
    bool rotating;
    bool start_rotate;
    float start_angle;
    public:
    const boost::filesystem::path source_path;
};

SonarScanItem::SonarScanItem(boost::filesystem::path source_path_,
                             QGraphicsItem *parent) :
    QGraphicsPixmapItem(parent),
    rotating(false),
    source_path(source_path_) {

}

void
SonarScanItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    if (rotating && event->button() == Qt::LeftButton) {
        start_rotate = false;
        return;
    }
    QGraphicsPixmapItem::mousePressEvent(event);
    if (event->button() == Qt::RightButton) {
        rotating = true;
        start_rotate = true;
        curr_transform = transform();
        new_transform = curr_transform;
        start_point = event->scenePos();
        rotate_center = event->pos();
    }
}

void
SonarScanItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsPixmapItem::mouseMoveEvent(event);
    if (rotating) {
        auto delta = start_point - event->scenePos();
        if (start_rotate) {
            start_angle = std::atan2(delta.y(), delta.x()); 
        } else {
            new_transform = curr_transform;
            auto center = rotate_center;
            new_transform.translate(center.x(), center.y());
            float new_angle = std::atan2(delta.y(), delta.x());
            float rotation; 
            if (std::abs(new_angle - start_angle) > std::abs(start_angle - new_angle)) {
                rotation = start_angle - new_angle;
            } else {
                rotation = new_angle - start_angle;
            }
            new_transform.rotateRadians(rotation);
            new_transform.translate(-center.x(), -center.y());
            setTransform(new_transform);
        }
    }
}

void
SonarScanItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsPixmapItem::mouseReleaseEvent(event);
    if (event->button() == Qt::RightButton) {
        setTransform(new_transform);
        rotating = false;
    }
}

QGraphicsItem *cauv::image_to_scan_item(boost::filesystem::path file,
                                        boost::filesystem::path pose_file) {
    using namespace cauv;
    std::cout << file << std::endl;
    cauv_slam::SonarImage image;
    load_sonar_image(file, image);
    std::cout << "Range: " << image.rangeEnd << "(" << image.rangeConversion << "m per bin)" << std::endl;
    auto mapping = get_polar_mapping(image);
    auto map_mat = get_polar_to_cartesian_map(mapping, 10);
    auto mat = sonar_msg_to_mat(image);
    cv::Mat cartesian_mat(map_mat.rows, map_mat.cols, CV_8UC1);
    cv::remap(mat, cartesian_mat, map_mat, cv::Mat(), CV_INTER_LINEAR);

    auto item = new SonarScanItem(file);
    item->setPixmap(QPixmap::fromImage(sonar_mat_to_qimage(cartesian_mat)));
    item->setFlags(QGraphicsItem::ItemIsMovable | 
                   QGraphicsItem::ItemIsSelectable | 
                   QGraphicsItem::ItemDoesntPropagateOpacityToChildren);
    if (boost::filesystem::is_regular_file(pose_file)) {
        cv::FileStorage pose_store(pose_file.c_str(), cv::FileStorage::READ);
        cv::FileNode scans = pose_store["scans"];
        cv::FileNode pose_struct = scans[file.stem().c_str()];
        auto transform = item->transform();
        transform.translate((float)pose_struct["x"], (float)pose_struct["y"]);
        transform.rotateRadians((float)pose_struct["rotation"]);
        item->setTransform(transform);
    }
    return item;
}

void cauv::save_scan_poses(const std::vector<QGraphicsItem*> &scans,
                           const boost::filesystem::path pose_file) {
    cv::FileStorage pose_store(pose_file.c_str(), cv::FileStorage::WRITE);
    pose_store << "scans" << "{";
    for (auto &item: scans) {
        auto scan_item = dynamic_cast<SonarScanItem*>(item);
        if (!scan_item) {
            std::cout << "Non-SonarScanItem scan item " << item << "? WTF!" << std::endl;
            continue;
        }
        pose_store << scan_item->source_path.stem().c_str() << "{";
        auto transform = item->transform();
        auto point_a = transform.map(QPointF(0,0));
        auto point_b = transform.map(QPointF(1,0));
        auto delta = point_b - point_a;
        auto translate = point_a + item->pos();
        pose_store << "rotation" << std::atan2(delta.y(), delta.x());
        pose_store << "x" << translate.x();
        pose_store << "y" << translate.y();
        pose_store << "}";
    }
    pose_store << "}";
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    cauv::Options options("Manual positioning of sonar images");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("directory,d", po::value<std::string>()->required(), "Directory of sonar images to load")
        ("poses,p", po::value<std::string>(), "Pose file to view/edit/create")
        ("start,s", po::value<int>()->default_value(0), "Start frame")
        ("end,e", po::value<int>()->default_value(10000000), "End frame")
      ;
    if (options.parseOptions(argc, argv)) {
        return 0;
    };

    auto vm = options.vm;

    auto image_directory = vm["directory"].as<std::string>();

    std::cout << vm["directory"].as<std::string>() << std::endl;
    int start_frame = vm["start"].as<int>();
    int end_frame = vm["end"].as<int>();
    using namespace cauv;
    auto all_files = get_msg_files(image_directory);
    std::vector<boost::filesystem::path> files;
    int ii = 0;
    for (auto &file: all_files) {
        ii++;
        if (ii > start_frame && ii < end_frame) {
            files.push_back(file);
        }
    }
    boost::filesystem::path pose_file;
    if (vm.count("poses")) {
        pose_file = vm["poses"].as<std::string>();
    }

    auto scene = new QGraphicsScene();
    scene->setBackgroundBrush(Qt::black);
    auto widget = new ManualLayoutView(files, pose_file);

    widget->setScene(scene);
    widget->show();
    app.exec();

    return 0;
}
