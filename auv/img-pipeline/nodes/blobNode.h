#ifndef __BLOB_NODE_H__
#define __BLOB_NODE_H__
#include "../node.h"
#include <cvblob.h>

namespace cauv{
namespace imgproc{

class BlobNode: public Node{
    public:
        BlobNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;

            registerInputID("channel (not copied)"); // must be matrix of a single channel

            registerOutputID("blobs", std::vector<Ellipse>());

            registerParamID<int>("expand_size",5);
            registerParamID<int>("min_area",100);
        }

    protected:
        struct findBlobs: boost::static_visitor<cvb::CvBlobs> {
            public:
            cvb::CvBlobs operator()(const cv::Mat mat) const {
                return getBlobs(mat);
            }
            cvb::CvBlobs operator()(const NonUniformPolarMat pmat) const {
                return getBlobs(pmat.mat);
            }
            cvb::CvBlobs operator()(const PyramidMat pymat) const {
                throw parameter_error("pyramidal images not supported");
            }
            private:
            cvb::CvBlobs getBlobs(cv::Mat mat) const {
                cv::Mat labelimg(mat.size(), cv::DataType<cvb::CvLabel>::type);
                IplImage label_ipl = labelimg;
                //IplImage *labelimg = cvCreateImage(img.size(), IPL_DEPTH_LABEL, 1);
                IplImage in_image = mat;
                cvb::CvBlobs blobs;
                cvb::cvLabel(&in_image, &label_ipl, blobs);
                return blobs;
            }
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){
            cvb::CvBlobs blobs;
            try {
                augmented_mat_t m = inputs["channel (not copied)"]->augmentedMat();
                blobs = boost::apply_visitor(findBlobs(), m);
            } catch(cv::Exception& e) {
                error() << "BlobNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            cvb::cvFilterByArea(blobs, param<int>("min_area"), 100000);
            std::vector<Ellipse> kps;
            kps.reserve(blobs.size());
            typedef std::pair<cvb::CvLabel, cvb::CvBlob*> blob_pair;
            foreach(blob_pair p, blobs) {
                cvb::CvBlob b = *p.second;
                unsigned int width = b.maxx - b.minx;
                unsigned int height = b.maxy - b.miny;
                if (width > height) {
                    kps.push_back(Ellipse(floatXY(b.centroid.x, b.centroid.y), width, height, M_PI/2));
                } else {
                    kps.push_back(Ellipse(floatXY(b.centroid.x, b.centroid.y), height, width, 0));
                }
            }
            cvb::cvReleaseBlobs(blobs);
            r["blobs"] = kps;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv


#endif
