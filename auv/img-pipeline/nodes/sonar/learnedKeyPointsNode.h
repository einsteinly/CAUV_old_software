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

#include <list>
#include <vector>
#include <cassert>
#include <cmath>
#include <limits>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>

namespace cauv{
namespace imgproc{

typedef std::vector<cv::Point> pt_vec;
typedef std::vector<int> int_vec;
typedef std::vector<std::size_t> idx_vec;

// - Random Forest Classes
class RFQuestion{
    public:
        /* Apply this question to the data.
         */
        virtual void apply(pt_vec const& keypoints,
                          cv::Mat const& image,
                          std::vector<int>& answers) const{
            for(int i = 0; i < keypoints.size(); i++)
                answers[i] = apply(keypoints[i], image);
        }

        virtual float entropyOverTarget(pt_vec const& keypoints, 
                                        int_vec const& target,
                                        cv::Mat const& image) const{
            unsigned good_true = 0;
            unsigned good_false = 0;
            unsigned bad_true = 0;

            assert(keypoints.size() == target.size());

            for(int i = 0; i < keypoints.size(); i++){
                if(apply(keypoints[i], image)){
                    good_true += target[i];
                    bad_true += !target[i];
                }else{
                    good_false += !target[i];
                }
            }

            const unsigned n = keypoints.size();
            const unsigned bad_false = n - (good_true + good_false + bad_true);
            
            //const unsigned ngood = good_true + good_false;
            //const unsigned nbad = n - good;
            const unsigned ntrue = good_true + bad_true;
            const unsigned nfalse = n - ntrue;

            if(ntrue == 0 || nfalse == 0)
                // this question tells us nothing!
                return std::numeric_limits<float>::infinity();
            
            // entropy of returned sets (summed)
            return  -(good_true * good_true * std::log(float(good_true)/ntrue) +
                      bad_true * bad_true * std::log(float(bad_true)/ntrue)) / ntrue
                    -(good_false * good_false * std::log(float(good_false)/nfalse) +
                      bad_false * bad_false * std::log(float(bad_false)/nfalse)) / nfalse;
        }

        virtual bool apply(cv::Point const& pt, cv::Mat const& image) const = 0;

    protected:
        inline static bool inImage(cv::Point const& p, cv::Mat const& image){
            if(p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows)
                return true;
            return false;
        }
};

class PxRatioQuestion: public RFQuestion{
    public:
        PxRatioQuestion(cv::Point px_a_offset, cv::Point px_b_offset, float difference)
            : RFQuestion(), m_pt_a(px_a_offset), m_pt_b(px_b_offset), m_diff(difference){
        }

        virtual bool apply(cv::Point const& pt, cv::Mat const& image) const{
            cv::Point a = pt + m_pt_a;
            cv::Point b = pt + m_pt_b;
            if(inImage(a, image) &&
               inImage(b, image) &&
               image.at<uint8_t>(a) > image.at<uint8_t>(b) + m_diff)
                return true;
            return false;
        }

    private:
        const cv::Point m_pt_a;
        const cv::Point m_pt_b;
        const float     m_diff;
};

class TreeNode;
typedef boost::shared_ptr<TreeNode> TreeNode_ptr;

class TreeNode{
    public:
        TreeNode(RFQuestion const& question, bool terminal)
            : m_question(question), m_terminal(terminal){
        }
        
        // for terminal nodes only
        void setTrueGoodness(bool good){
            assert(m_terminal);
            m_true_side = TreeNode_ptr((TreeNode*)good);
            m_false_side = TreeNode_ptr((TreeNode*)!good);
        }

        void setTrueSide(TreeNode_ptr p){
            assert(!m_terminal);
            m_true_side = p;
        }
        void setFalseSide(TreeNode_ptr p){
            assert(!m_terminal);
            m_false_side = p;
        }

        inline bool test(cv::Point const& pt, cv::Mat const& image){
            if(m_question.apply(pt, image)){
                if(m_terminal){
                    return m_true_side; // pointer non-zero (but invalid) if good, zero if bad
                }else{
                    return m_true_side->test(pt, image);                
                }
            }else{
                if(m_terminal){
                    return m_false_side; // pointer non-zero (but invalid) if good, zero if bad
                }else{
                    return m_false_side->test(pt, image);
                }
            }
        }

    private:
        RFQuestion const& m_question;
        TreeNode_ptr m_false_side;
        TreeNode_ptr m_true_side;
        const bool m_terminal;
};

class ListOfRandomRFQuestions{
    private:
        typedef std::list< boost::shared_ptr<RFQuestion> > qlist_t;

    public:
        ListOfRandomRFQuestions(unsigned number_of_questions)
            : m_questions(){
            for(unsigned i = 0; i < number_of_questions; i++)
                m_questions.push_back(_randomPxRatioQuestion());
        }
        
        RFQuestion const& bestQuestionFor(pt_vec const& points,
                                          idx_vec const& pt_indices,
                                          int_vec const& goodness,
                                          cv::Mat const& image,
                                          bool& terminal,
                                          idx_vec& test_true,
                                          idx_vec& test_false) const{
            // !!! TODO: EDITING HERE, update this function for new signature

            qlist_t::const_iterator best = m_questions.begin();
            float best_entropy = std::numeric_limits<float>::infinity();
            for(qlist_t::const_iterator i = m_questions.begin(); i != m_questions.end(); i++){
                const float entropy = (*i)->entropyOverTarget(points, goodness, image);
                if(entropy < best_entropy){
                    best = i;
                    best_entropy = entropy;
                }
            }
            // !!! TODO: handle case where no questions could distinguish at
            // all between keypoints.
            assert(best != m_questions.end());
            return **best;
        }

    private:
        boost::shared_ptr<PxRatioQuestion> _randomPxRatioQuestion(){
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO: actual random questions
            return boost::make_shared<PxRatioQuestion>(cv::Point(0,0), cv::Point(2,2), 2.0);
        }

        qlist_t m_questions;
};

// - LearnedKeyPointsNode

/* 
 */
class LearnedKeyPointsNode: public Node{
    private:
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
            image_ptr_t training_img = inputs["training: keypoints image"];
            
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

        class TreeGrowingVisitor: public boost::static_visitor<TreeNode_ptr>{
            public:
                TreeGrowingVisitor(pt_vec const& points,
                                  int_vec const& goodness,
                                  ListOfRandomRFQuestions const& questions)
                    : m_points(points),
                      m_point_goodness(goodness),
                      m_questions(questions){
                }
                TreeNode_ptr operator()(cv::Mat a) const{
                    return _trainTreeTrunk(a);
                }
                TreeNode_ptr operator()(NonUniformPolarMat a) const{
                    return _trainTreeTrunk(a.mat);
                }
                TreeNode_ptr operator()(PyramidMat) const{
                    error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                    return TreeNode_ptr();
                }

            private:
                TreeNode_ptr _trainTreeTrunk(cv::Mat const& m) const{
                    idx_vec indices; indices.reserve(m_points.size());
                    for(size_t i = 0; i < m_points.size(); i++)
                        indices.push_back(i);
                    return _trainTree(m, m_points, indices, m_point_goodness);
                }
                inline TreeNode_ptr _trainTree(cv::Mat const& m,
                                               pt_vec const& points,
                                               idx_vec const& pt_indices,
                                               int_vec const& goodness) const{
                    bool terminal = false;
                    idx_vec test_true; test_true.reserve(0.8*points.size());
                    idx_vec test_false; test_false.reserve(0.8*points.size());
                    RFQuestion const& best_question = m_questions.bestQuestionFor(
                        points, pt_indices, goodness, m, terminal, test_true, test_false
                    );

                    TreeNode_ptr r = boost::make_shared<TreeNode>(boost::cref(best_question), terminal);
                    if(terminal){
                        r->setTrueGoodness(goodness[test_true[0]]);
                    }else{
                        r->setTrueSide(_trainTree(m, points, test_true, goodness));
                        r->setFalseSide(_trainTree(m, points, test_false, goodness));
                    }
                }

                pt_vec const& m_points;
                int_vec const& m_point_goodness;
                ListOfRandomRFQuestions const& m_questions;
        };

        void _train(kp_vec const& keypoints, int_vec const& goodness, image_ptr_t img){
            debug() << keypoints.size() <<  "kps, " << goodness.size() << "values for training";
            
            pt_vec kps_as_points;
            kps_as_points.reserve(keypoints.size());
            { 
            debug d; // TEMP: sanity checking coordinate values
            d << "sanity check kp coordinates (should be integers):";
            foreach(KeyPoint const& kp, keypoints){
                d << kp.pt;
                kps_as_points.push_back(cv::Point(int(kp.pt.x), int(kp.pt.y)));
            }
            }
            
            augmented_mat_t m = img->augmentedMat();
            
            ListOfRandomRFQuestions questions(5);
            TreeNode_ptr new_tree = boost::apply_visitor(TreeGrowingVisitor(kps_as_points, goodness, questions), m);
            
        }

    private:
        std::list<TreeNode_ptr> m_forest;

    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef LEARNEDKEYPOINTS_NODE_H
