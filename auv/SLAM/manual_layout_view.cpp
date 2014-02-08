#include "manual_layout_view.h"
#include <QWheelEvent>
#include <QScrollBar>
#include <QGraphicsItem>

using namespace cauv;

ManualLayoutView::ManualLayoutView(std::vector<boost::filesystem::path> scans_,
                                   boost::filesystem::path pose_file_,
                                   QWidget *parent)
    : QGraphicsView(parent),
      dragging(false),
      lastMouseEvent(QEvent::None, QPoint(), Qt::NoButton, 0, 0),
      scan_files(scans_),
      scan_index(0),
      pose_file(pose_file_) {
    //setDragMode(ScrollHandDrag);
}

void ManualLayoutView::mousePressEvent(QMouseEvent *event) {
    QGraphicsView::mousePressEvent(event);
    if (event->button() == Qt::MiddleButton) {
        event->accept();
        dragging = true;
        viewport()->setCursor(Qt::ClosedHandCursor);
    }
}

void ManualLayoutView::mouseMoveEvent(QMouseEvent *event) {
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

void ManualLayoutView::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);

    if (dragging && event->button() == Qt::MiddleButton) {
        viewport()->setCursor(Qt::OpenHandCursor);
        dragging = false;
    }
}

void ManualLayoutView::wheelEvent(QWheelEvent *event) {
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    static const double scaleFactor = 1.1;
    if (event->delta() > 0) {
        scale(scaleFactor, scaleFactor);
    } else {
        scale(1 / scaleFactor, 1 / scaleFactor);
    }
}

void ManualLayoutView::addScan() {
    auto &path = scan_files.at(scan_index++);
    auto item = image_to_scan_item(path, pose_file);
    if (scan_items.size()) {
        item->setParentItem(scan_items.back());
    } else {
        scene()->addItem(item);
    }
    scan_items.push_back(item);
    float opacity = 1;
    for (int i = scan_items.size() - 1; i >= 0; i--) {
        if (!scan_items[i]->scene()) {
            continue;
        }
        scan_items[i]->setOpacity(opacity);
        if (opacity * 255 < 1) {
            scan_items[i + 1]->setParentItem(NULL);
            scene()->removeItem(scan_items[i]);
        }
        opacity *= 1;
    }
}

void ManualLayoutView::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Right) {
        addScan();
    }
    if (event->key() == Qt::Key_S) {
        if (!pose_file.empty()) {
            save_scan_poses(scan_items, pose_file);
        }
    }
}
