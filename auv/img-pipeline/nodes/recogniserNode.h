#ifndef __RECOGNISER_NODE_H__
#define __RECOGNISER_NODE_H__

#include "../node.h"

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/features2d/features2d.hpp"

namespace cauv{
namespace imgproc{

class RecogniserNode: public Node{
    public:
        RecogniserNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = slow;

            // input:
            //registerParamID< std::vector<KeyPoint> >("Reference KeyPoints", std::vector<KeyPoint>(), "");
            registerInputID("Reference Image", May_Be_Old);
            //registerParamID< std::vector<KeyPoint> >("Image KeyPoints", std::vector<KeyPoint>(), "", Must_Be_New);
            registerInputID("Image");

            // outputs:
            registerOutputID<NodeParamValue>("Confidence");
            registerOutputID<NodeParamValue>("Position");
            registerOutputID<image_ptr_t>("Correspondence Image");

            // Parameters:
            //registerParamID<std::string>("algorithm", "FERN", "one of FAST STAR SIFT SURF MSER GFTT HARRIS")
            registerParamID<float>("SURF Hessian threshold", 5.0e3);
            registerParamID<int>("FERN nclasses", 0, "");
            registerParamID<int>("FERN patchSize", 31, "");
            registerParamID<int>("FERN signatureSize", INT_MAX, "");
            registerParamID<int>("FERN nstructs", 50, "");
            registerParamID<int>("FERN structSize", 9, "");
            registerParamID<int>("FERN nviews", 1000, "");
            registerParamID<int>("FERN compressionMethod", 0, "");
        }

        virtual ~RecogniserNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            //std::vector<KeyPoint> ref_kps = param< std::vector<KeyPoint> >("Reference KeyPoints");
            //std::vector<cv::KeyPoint> img_kps = _cvKeyPointVec(param< std::vector<KeyPoint> >("Image KeyPoints"));
            //m_reference_keypoints = conv(ref_kps);

            image_ptr_t ref_img = inputs["Reference Image"];
            image_ptr_t img = inputs["Image"];
            float surf_hess_thr = param<float>("SURF Hessian threshold");
                
            cv::FernDescriptorMatcher::Params dm_params(
                param<int>("FERN nclasses"), param<int>("FERN patchSize"),
                param<int>("FERN signatureSize"), param<int>("FERN nstructs"),
                param<int>("FERN structSize"), param<int>("FERN nviews"),
                param<int>("FERN compressionMethod")
            );
            cv::FernDescriptorMatcher dm = cv::FernDescriptorMatcher(dm_params);

            std::vector<cv::KeyPoint> kp2;
            std::vector<cv::KeyPoint> kp1;

            cv::SURF surf_extractor(surf_hess_thr);
            
            const cv::Mat mask;
            surf_extractor(ref_img->cvMat(), mask, kp1); 
            debug() << *this << kp1.size() <<  "keypoints from reference image";

            surf_extractor(img->cvMat(), mask, kp2);
            debug() << *this << kp2.size() <<  "keypoints from image";
            
            if(!kp1.size())
                throw std::runtime_error("No keypoints detected in reference image");
            if(!kp2.size())
                throw std::runtime_error("No keypoints detected in image");

            // find NN for each of kp2 in kp1
            cv::vector<cv::DMatch> matches2to1;
            debug() << *this << "Fern Descriptor Matcher Working...";
            dm.match(img->cvMat(), kp2, ref_img->cvMat(), kp1, matches2to1);
            debug() << *this << matches2to1.size() << "Descriptor Matches";
            printf("Done\n");
            
            r["Correspondence Image"] = _drawCorresp(ref_img, kp1, img, kp2, matches2to1);
            return r;
        }
        
        std::vector<cv::KeyPoint> _cvKeyPointVec(std::vector<KeyPoint> const& v){
            std::vector<cv::KeyPoint> r;
            r.reserve(v.size());
            std::vector<KeyPoint>::const_iterator i;
            for(i = v.begin(); i != v.end(); i++)
                r.push_back(_cvKeyPoint(*i));
            return r;
        }
    
    private:
        image_ptr_t _drawCorresp(image_ptr_t im1, std::vector<cv::KeyPoint> im1_kp,
                                 image_ptr_t im2, std::vector<cv::KeyPoint> im2_kp,
                                 std::vector<cv::DMatch> const& desc_idx){
            IplImage  img1_d = IplImage(im1->cvMat());
            IplImage* img1   = &img1_d;
            IplImage  img2_d = IplImage(im2->cvMat());
            IplImage* img2   = &img2_d;

            IplImage* img_corr = cvCreateImage(cvSize(img1->width + img2->width, MAX(img1->height, img2->height)),
                                               IPL_DEPTH_8U, 3);
            CvScalar z = {{0.0, 0.0, 0.0, 0.0}};
            cvSet(img_corr, z);

            cvSetImageROI(img_corr, cv::Rect(0, 0, img1->width, img1->height));
            cvCvtColor(img1, img_corr, CV_GRAY2RGB);
            cvSetImageROI(img_corr, cv::Rect(img1->width, 0, img2->width, img2->height));
            cvCvtColor(img2, img_corr, CV_GRAY2RGB);
            cvResetImageROI(img_corr);

            for (size_t i = 0; i < im1_kp.size(); i++)
            {
                cvCircle(img_corr, im1_kp[i].pt, 3, CV_RGB(255, 0, 0));
            }

            for (size_t i = 0; i < im2_kp.size(); i++)
            {
                CvPoint pt = cvPoint(cvRound(im2_kp[i].pt.x + img1->width), cvRound(im2_kp[i].pt.y));
                cvCircle(img_corr, pt, 3, CV_RGB(255, 0, 0));
                cvLine(img_corr, im1_kp[desc_idx[i].trainIdx].pt, pt, CV_RGB(0, 255, 0));
            }
            return boost::make_shared<Image>(cv::Mat(img_corr, true));
        }
        
        static cv::KeyPoint _cvKeyPoint(cauv::KeyPoint const& kp){ 
            return cv::KeyPoint(kp.pt.x, kp.pt.y, kp.size, kp.angle,
                                kp.response, kp.octave, kp.class_id);
        }

        //std::vector<cv::KeyPoint> m_reference_keypoints;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RECOGNISER_NODE_H__


