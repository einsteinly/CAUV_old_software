#ifndef __BLOB_NODE_H__
#define __BLOB_NODE_H__

#include "../node.h"
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/Circle.h>

namespace cauv{
namespace imgproc{

class BlobNode: public Node{
    public:
        BlobNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;

            registerInputID("image (not copied)"); // must be matrix of a single channel

            registerOutputID("blobs", std::vector<Circle>());

            registerParamID<Range>("threshold",Range(20,240));
            registerParamID<int>("threshold_step",10);
            registerParamID<Range>("area",Range(100,8000));
            registerParamID<Range>("circularity",Range(0.8,std::numeric_limits<float>::infinity()));
            registerParamID<Range>("inertiaRatio",Range(0.1,std::numeric_limits<float>::infinity()));
            registerParamID<Range>("convexity",Range(0.95,std::numeric_limits<float>::infinity()));
            registerParamID<bool>("find_dark",true);
            registerParamID<bool>("find_light",true);

        }

    protected:
        struct findBlobs: boost::static_visitor< std::vector<cv::KeyPoint> > {
            public:
            findBlobs(const cv::SimpleBlobDetector& detector) : detector(detector) {
            }

            std::vector<cv::KeyPoint> operator()(const cv::Mat mat) const {
                return getBlobs(mat);
            }
            std::vector<cv::KeyPoint> operator()(const NonUniformPolarMat pmat) const {
                return getBlobs(pmat.mat);
            }
            std::vector<cv::KeyPoint> operator()(const PyramidMat) const {
                throw parameter_error("pyramidal images not supported");
            }
            private:
            std::vector<cv::KeyPoint> getBlobs(cv::Mat mat) const {
                std::vector<cv::KeyPoint> blobs;
                detector.detect(mat, blobs);
                return blobs;
            }
            private:
            const cv::SimpleBlobDetector& detector;
        };

        struct getDimensions : boost::static_visitor< std::pair <int, int> > {
            public:
            std::pair<int, int> operator()(const cv::Mat mat) const {
                return std::make_pair(mat.cols, mat.rows);
            }
            std::pair<int, int> operator()(const NonUniformPolarMat pmat) const {
                return operator()(pmat.mat);
            }
            std::pair<int, int> operator()(const PyramidMat) const {
                throw parameter_error("pyramedal images not suppored");
            }
        };

        void doWork(in_image_map_t& inputs, out_map_t& r){
            bool find_dark = param<bool>("find_dark");
            bool find_light = param<bool>("find_light");

            if (!find_dark && !find_light)
                return;

            cv::SimpleBlobDetector::Params blobDetectorParams;
            blobDetectorParams.filterByArea = param<Range>("area").min > 0 || param<Range>("area").max < std::numeric_limits<float>::infinity();
            blobDetectorParams.minArea = param<Range>("area").min;
            blobDetectorParams.maxArea = param<Range>("area").max;
            blobDetectorParams.filterByCircularity = param<Range>("circularity").min > 0 || param<Range>("circularity").max < std::numeric_limits<float>::infinity();
            blobDetectorParams.minCircularity = param<Range>("circularity").min;
            blobDetectorParams.filterByInertia = param<Range>("inertiaRatio").min > 0 || param<Range>("inertiaRatio").max < std::numeric_limits<float>::infinity();
            blobDetectorParams.minInertiaRatio = param<Range>("inertiaRatio").min;
            blobDetectorParams.filterByConvexity = param<Range>("convexity").min > 0 || param<Range>("convexity").max < std::numeric_limits<float>::infinity();
            blobDetectorParams.minConvexity = param<Range>("convexity").min;
            #if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION > 2
            blobDetectorParams.maxCircularity = param<Range>("circularity").max;
            blobDetectorParams.maxInertiaRatio = param<Range>("inertiaRatio").max;
            blobDetectorParams.maxConvexity = param<Range>("convexity").max;
            #endif
            if (find_dark && find_light)
                blobDetectorParams.filterByColor = false;
            else {
                blobDetectorParams.filterByColor = true;
                blobDetectorParams.blobColor = find_dark ? 0 : 255;
            }

            cv::SimpleBlobDetector blobDetector(blobDetectorParams);

            std::vector<cv::KeyPoint> blobs;
            float width = 1;
            float height = 1;
            try {
                augmented_mat_t m = inputs["image (not copied)"]->augmentedMat();
                std::pair<int, int> dimensions = boost::apply_visitor(getDimensions(), m);
                width = dimensions.first;
                height = dimensions.second;
                blobs = boost::apply_visitor(findBlobs(blobDetector), m);
            } catch(cv::Exception& e) {
                error() << "BlobNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
                return;
            }
            std::vector<Circle> circles;
            circles.reserve(blobs.size());
            foreach(const cv::KeyPoint& b, blobs) {
                circles.push_back(Circle(floatXY(b.pt.x / width,
                                                 b.pt.y / height),
                                                 b.size / ((width + height) / 2)));
            }
            r["blobs"] = circles;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv


#endif
