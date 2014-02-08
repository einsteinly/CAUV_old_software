#pragma once
#include <QGraphicsView>
#include <QMouseEvent>
#include <boost/filesystem.hpp>
#include <vector>

namespace cauv {

//These two functions are implemented in manual_layout.cpp
QGraphicsItem *image_to_scan_item(boost::filesystem::path file,
                                  boost::filesystem::path pose_file);

void save_scan_poses(const std::vector<QGraphicsItem*> &scan_items,
                     const boost::filesystem::path pose_file);

class ManualLayoutView : public QGraphicsView {
    public:
    ManualLayoutView(std::vector<boost::filesystem::path> scans, 
                     boost::filesystem::path pose_file, 
                     QWidget *parent = NULL);

    void loadItems();

    protected:
    virtual void wheelEvent(QWheelEvent *event);

    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);

    private:
    void addScan();
    bool dragging;
    QMouseEvent lastMouseEvent;
    std::vector<boost::filesystem::path> scan_files;
    std::vector<QGraphicsItem*> scan_items;
    int scan_index;

    boost::filesystem::path pose_file;
};

}
