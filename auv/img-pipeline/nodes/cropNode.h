#ifndef __CROP_NODE_H__
#define __CROP_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


//Used for cropping (self-explanatory) and anti-cropping (ask James Crosby)

namespace cauv{
namespace imgproc{

class CropNode: public Node{
    public:
        CropNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID(Image_In_Name);
            
            // output:
            registerOutputID<image_ptr_t>(Image_Out_Name);
            
            // multiple parameters
            registerParamID<int>("top left (x)",0);
            registerParamID<int>("top left (y)",0);
            registerParamID<int>("width", 1);
            registerParamID<int>("height", 1);
        }
    
        virtual ~CropNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            try{
                int top_left_x = param<int>("top left (x)");
                int top_left_y = param<int>("top left (y)");
                int width = param<int>("width");
                int height = param<int>("height");
            
                if (width < 0 || height < 0) {
                    error() << "CropNode:\n\t"
                            << "width and height must be greater than 0.";
                    return r;
                }
                image_ptr_t inp_img = inputs[Image_In_Name];
                if (top_left_x >= inp_img->cvMat().cols || top_left_y >= inp_img->cvMat().rows ||   // Whole of image is being cropped
                    width < -top_left_x || height < -top_left_y) {
                    error() << "CropNode:\n\t"
                             << "whole of image is being cropped."; 
                    return r;
                }   

                   cv::Rect cropRect(top_left_x,top_left_y, width, height); //Create the rectangle used to crop the picture

                image_ptr_t dst;
                // Check if we need to perform anti-cropping
                if (top_left_x < 0 || top_left_y < 0 ||
                    top_left_x + width > inp_img->cvMat().cols || top_left_y + height > inp_img->cvMat().rows) 
                { // We need to perform anti-cropping
                    // The cropping rectangle must fit on the image                   
                    if(cropRect.x < 0) {  // The left hand side of the rectangle falls to the left of the image, adjust accordingly 
                        cropRect.x = 0;
                        cropRect.width += top_left_x; 
                    }
                    if(cropRect.x + cropRect.width > inp_img->cvMat().cols) { // The right hand side of the rectange falls to the right of the image, adjust accordindly
                        cropRect.width = inp_img->cvMat().cols - cropRect.x;                    
                    }
                     
                    if(cropRect.y < 0) {  // The rectangle falls to the top of the image, adjust accordingly 
                        cropRect.y = 0;
                        cropRect.height += top_left_y; 
                    }
                    if(cropRect.y + cropRect.height > inp_img->cvMat().rows) { // The rectange falls to the bottom of the image, adjust accoringly
                        cropRect.height = inp_img->cvMat().rows - cropRect.y;                    
                    }

                    
                    cv::Mat cropped_img = cv::Mat(inp_img->cvMat(),cropRect); //Perform the cropping
                    
                    cv::Mat out_img = cv::Mat::zeros(height,width,inp_img->cvMat().type()); //Create a new blank image
                    // We need to placed the cropped-image into the output matrix. We can use cropRect for this, translated as required
                    if(top_left_x < 0) { cropRect.x = - top_left_x; } else { cropRect.x = 0; }
                    if(top_left_y < 0) { cropRect.y = - top_left_y; } else { cropRect.y = 0; }
                    // Form a submatrix of the output matrix where we want to put the cropped image and then copy the image
                    //error() << "x: " << cropRect.x << " y: " << cropRect.y << " width: " << cropRect.width << " height: " << cropRect.height << " m cols: " << out_img.cols << " m rows: " << out_img.rows; // << " m width: " << width << " m height: " << height;
                    cv::Mat out_img_sub = cv::Mat(out_img, cropRect);
                    cropped_img.copyTo(out_img_sub);
                    dst = boost::make_shared<Image>(out_img);
                } else { // Simply crop
                    cv::Mat cropped_img = cv::Mat(inp_img->cvMat(),cropRect); //Perform the cropping
                    dst = boost::make_shared<Image>(cropped_img);
                }
                r[Image_Out_Name] = dst;
                
        
            } catch(cv::Exception& e) {
                error() << "CropNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            return r;
        

         
/*
        try{
            cv::Mat cropped_img = cv::Mat(img->cvMat(),cropRect); //Perform the cropping
            //See if we need anti-cropping
            if (top_left_x < 0 || top_left_y < 0 || top_left_x + width > img->cvMat().cols || top_left_y + height > img->cvMat().rows) {
                cv::Rect dst_rec(-top_left_x,-top_left_y, cropRect.width, cropRect.height); //create a rectangle where the cropped image should be placed in the new image
                if(dst_rec.x < 0) { dst_rec.x = 0; }
                if(dst_rec.y < 0) { dst_rec.y = 0; }
                error() << "x: " << dst_rec.x << " y: " << dst_rec.y << " width: " << dst_rec.width << " height: " << dst_rec.height << " m cols: " << dst->cvMat().cols << " m rows: " << dst->cvMat().rows << " m width: " << width << " m height: " << height;
                cv::Mat roi(dst->cvMat(),dst_rec);
                cropped_img.copyTo(roi);    
                
        } else { cropped_img.copyTo(dst->cvMat()); }

        error() << "width: " << dst->cvMat().cols << "\n\t";
                r["cropped image"] = dst; 
            }catch(cv::Exception& e){
                error() << "CropNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
                return r;
            }
            
            return r; */
        }
        
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CROP_NODE_H__

