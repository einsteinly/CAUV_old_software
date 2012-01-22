/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef LEARNEDKEYPOINTS_NODE_H
#define LEARNEDKEYPOINTS_NODE_H

namespace cauv{
namespace imgproc{

/*
 */
class LearnedKeyPointsNode: public Node{
    private:
        typedef std::vector<int> int_vec;
        typedef std::vector<cauv::KeyPoint> kp_vec;

    public:
        LearnedKeyPointsNode(ConstructArgs const& args)
            : Node(args)
        {
        }

        virtual void init(){
            m_speed = fast;
            
            // source image input:
            registerInputID("image");
            registerParamID("boostrap keypoints", kp_vec());

            // training inputs:
            // These must be set from the same node
            registerParamID("training: keypoints", kp_vec(), "keypoints to update training data", May_Be_Old);
            registerInputID("training: keypoints image", Optional);            
            registerParamID("training: goodness", int_vec(), "score values to update training data", May_Be_Old);

            // outputs
            registerOutputID("keypoints", kp_vec());
            registerOutputID("image");
        }
    
    protected:
    
        struct ExtractKeyPoints: boost::static_visitor< kp_vec >{
            ExtractKeyPoints(){
            }
            kp_vec operator()(cv::Mat) const{
                error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                return kp_vec();
            }
            kp_vec operator()(NonUniformPolarMat) const{
                error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                return kp_vec();
            }
            kp_vec operator()(PyramidMat) const{
                error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                return kp_vec();
            }
            private:
        };

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];

            kp_vec training_keypoints = param< kp_vec >("training: keypoints");
            int_vec training_goodness = param< int_vec >("training: goodness");
            image_ptr_t training_img = inputs["training:keypoints image"];
            
            if(training_keypoints.size() && training_img){
                _train(training_keypoints, training_goodness, training_img);
            }
            
            bool bootstrap = true;
            
            if(!bootstrap){
                kp_vec kps;
                try{
                    augmented_mat_t m = img->augmentedMat();
                    kps = boost::apply_visitor(ExtractKeyPoints(), m);
                }catch(cv::Exception& e){
                    error() << "LearnedKeyPointsNode:\n\t"
                            << e.err << "\n\t"
                            << "in" << e.func << "," << e.file << ":" << e.line;
                }
                r["keypoints"] = kps;
            }else{
                r["keypoints"] = param< kp_vec >("boostrap keypoints");
            }

            r["image"] = img;

            return r;
        }

        void _train(kp_vec const& keypoints, int_vec const& goodness, image_ptr_t img){
            debug() << keypoints.size() <<  "kps, " << goodness.size() << "values for training";
            
        }

    private:
        

    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef LEARNEDKEYPOINTS_NODE_H
