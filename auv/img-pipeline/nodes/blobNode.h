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
        void doWork(in_image_map_t& inputs, out_map_t& r){
            cv::Mat img = inputs["channel (not copied)"]->mat();
            IplImage *labelimg = cvCreateImage(img.size(), IPL_DEPTH_LABEL, 1);
            IplImage in_image = img;
            cvb::CvBlobs blobs;
            const int max_value=255;
            try {
                cvb::cvLabel(&in_image, labelimg, blobs);
            }catch(cv::Exception& e){
                error() << "BlobNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            cvReleaseImage(&labelimg);
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
