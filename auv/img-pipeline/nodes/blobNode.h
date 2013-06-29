/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

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

            registerInputID("image (not copied)", NonConst); // must be matrix of a single channel

            registerOutputID("blobs", std::vector<Circle>());

            registerParamID<float>("threshold_min",20);
            registerParamID<float>("threshold_max",240);
            registerParamID<int>("threshold_step",10);
            registerParamID<float>("area_min",100);
            registerParamID<float>("area_max",8000);
            registerParamID<float>("circularity_max",std::numeric_limits<float>::infinity());
            registerParamID<float>("circularity_min",0.8);
            registerParamID<float>("inertiaRatio_min",0.1);
            registerParamID<float>("inertiaRatio_max",std::numeric_limits<float>::infinity());
            registerParamID<float>("convexity_min",0.95);
            registerParamID<float>("convexity_max",std::numeric_limits<float>::infinity());
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
            blobDetectorParams.filterByArea = param<float>("area_min") > 0 || param<float>("area_max") < std::numeric_limits<float>::infinity();
            blobDetectorParams.minArea = param<float>("area_min");
            blobDetectorParams.maxArea = param<float>("area_max");
            blobDetectorParams.filterByCircularity = param<float>("circularity_min") > 0 || param<float>("circularity_max") < std::numeric_limits<float>::infinity();
            blobDetectorParams.minCircularity = param<float>("circularity_min");
            blobDetectorParams.filterByInertia = param<float>("inertiaRatio_min") > 0 || param<float>("inertiaRatio_max") < std::numeric_limits<float>::infinity();
            blobDetectorParams.minInertiaRatio = param<float>("inertiaRatio_min");
            blobDetectorParams.filterByConvexity = param<float>("convexity_min") > 0 || param<float>("convexity_max") < std::numeric_limits<float>::infinity();
            blobDetectorParams.minConvexity = param<float>("convexity_min");
            #if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION > 2
            blobDetectorParams.maxCircularity = param<float>("circularity_max");
            blobDetectorParams.maxInertiaRatio = param<float>("inertiaRatio_max");
            blobDetectorParams.maxConvexity = param<float>("convexity_max");
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
            for (const cv::KeyPoint& b : blobs) {
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
