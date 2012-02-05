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
#if BOOST_VERSION >= 104700
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#else // old boost random hacks
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#warning !! please update your boost version to 1.47 or greater, this is way too hacky !!
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include<boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
namespace boost{
namespace random{
typedef boost::mt19937 mt19937;
template<typename T> struct uniform_real_distribution: boost::uniform_real<T>{
    uniform_real_distribution(T const& a, T const& b) : boost::uniform_real<T>(a,b){}
};
template<typename T> struct uniform_int_distribution: boost::uniform_int<T>{
    uniform_int_distribution(T const& a, T const& b) : boost::uniform_int<T>(a,b){}
};
} // namespace random
} // namespace boost
#endif // old boost random hacks
#include <boost/random/mersenne_twister.hpp>

#include <opencv2/core/core.hpp>

#include "utility/bash_cout.h"

namespace cauv{
namespace imgproc{

typedef std::vector<cv::Point> pt_vec;
typedef std::vector<int> int_vec;
typedef std::vector<std::size_t> idx_vec;
typedef boost::shared_ptr<idx_vec> idx_vec_ptr;

// !!! belongs in TreeNode really:
enum terminal_e {
    NotTerminal=0,
    TrueGood=1,
    TrueBad=2,
    TerminalTrue=3,
    FalseGood=4,
    FalseBad=8,
    TrueGoodFalseBad=9,
    TrueBadFalseGood=6,
    TerminalFalse=12,    
};

// - Random Forest Classes
class RFQuestion{
    public:
        typedef float entropy_t;

        /* Apply this question to the data.
         */
        virtual void apply(pt_vec const& points,
                          cv::Mat const& image,
                          std::vector<int>& answers) const{
            for(std::size_t i = 0; i < points.size(); i++)
                answers[i] = apply(points[i], image);
        }

        // test_true and test_false must be clear, the value of terminal
        // doesn't matter
        virtual entropy_t entropyOverTarget(pt_vec const& points,
                                            idx_vec const& pt_indices,
                                            int_vec const& target,
                                            cv::Mat const& image,
                                            int& terminal_flags,
                                            idx_vec_ptr test_true,
                                            idx_vec_ptr test_false) const{
            unsigned good_true = 0;
            unsigned good_false = 0;
            unsigned bad_true = 0;

            assert(points.size() == target.size());

            for(std::size_t i = 0; i < pt_indices.size(); i++){
                const size_t idx = pt_indices[i];
                if(apply(points[idx], image)){
                    assert(target[idx] == 1 || target[idx] == 0);
                    good_true += target[idx];
                    bad_true += !target[idx];
                    test_true->push_back(idx);
                }else{
                    good_false += target[idx];
                    test_false->push_back(idx);
                }
            }

            const unsigned n = pt_indices.size();
            const unsigned bad_false = n - (good_true + good_false + bad_true);
            
            //const unsigned ngood = good_true + good_false;
            //const unsigned nbad = n - good;
            const unsigned ntrue = good_true + bad_true;
            const unsigned nfalse = n - ntrue;
            assert(test_true->size() == ntrue);
            assert(test_false->size() == nfalse);
            
            // terminal if the question is perfect on either side:
            terminal_flags = 0;
            if(good_true && !bad_true)
                terminal_flags |= TrueGood;
            else if(bad_true && !good_true)
                terminal_flags |= TrueBad;

            if(bad_false && !good_false)
                terminal_flags |= FalseBad;
            else if(good_false && !bad_false)
                terminal_flags |= FalseGood;

            if(ntrue == 0 || nfalse == 0){
                // this question tells us nothing!
                debug(11) << this << "no information over" << n
                          << "points:" << good_true << good_false
                          << bad_true << bad_false;
                return std::numeric_limits<entropy_t>::infinity();
            }
            
            // entropy of returned sets: 0 * std::log(0) = NaN, hence the need
            // to check each part of the sum
            entropy_t r = 0;
            if(good_true)
                r -= (good_true * good_true * std::log(entropy_t(good_true)/ntrue)) / ntrue;
            if(bad_true)
                r -= (bad_true * bad_true * std::log(entropy_t(bad_true)/ntrue)) / ntrue;
            if(good_false)
                r -= (good_false * good_false * std::log(entropy_t(good_false)/nfalse)) / nfalse;
            if(bad_false)
                r -= (bad_false * bad_false * std::log(entropy_t(bad_false)/nfalse)) / nfalse;
            debug(11) << this << "entropy of" << r << "over" << pt_indices.size() << "points:"
                      << good_true << good_false << bad_true << bad_false << terminal_flags;
            return r;
        }

        virtual bool apply(cv::Point const& pt, cv::Mat const& image) const = 0;

    protected:
        inline static bool inImage(cv::Point const& p, cv::Mat const& image){
            if(p.x >= 0 && p.x < image.cols && p.y >= 0 && p.y < image.rows)
                return true;
            return false;
        }
};
typedef boost::shared_ptr<RFQuestion> Question_ptr;
typedef boost::shared_ptr<RFQuestion const> Question_constptr;

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
        TreeNode(Question_constptr question, int terminal_flags)
            : m_question(question),
              m_true_side((TreeNode*)(!!(terminal_flags & TrueGood)), null_deleter),
              m_false_side((TreeNode*)(!!(terminal_flags & FalseGood)), null_deleter),
              m_terminal(terminal_flags){
        }

        void setTrueSide(TreeNode_ptr p){
            assert(!(m_terminal & TerminalTrue));
            m_true_side = p;
        }
        void setFalseSide(TreeNode_ptr p){
            assert(!(m_terminal & TerminalFalse));
            m_false_side = p;
        }

        inline bool test(cv::Point const& pt, cv::Mat const& image) const{
            if(m_question->apply(pt, image)){
                if(m_terminal & TerminalTrue){
                    return m_true_side; // pointer non-zero (but invalid) if good, zero if bad
                }else{
                    return m_true_side->test(pt, image);                
                }
            }else{
                if(m_terminal & TerminalFalse){
                    return m_false_side; // pointer non-zero (but invalid) if good, zero if bad
                }else{
                    return m_false_side->test(pt, image);
                }
            }
        }

    private:
        static struct NullDeleter{
            void operator()(TreeNode*) const {}
        } null_deleter;

        Question_constptr m_question;
        TreeNode_ptr m_true_side;
        TreeNode_ptr m_false_side;
        const int m_terminal;
};

class ListOfRandomRFQuestions{
    private:
        typedef std::list<Question_ptr> qlist_t;
        typedef RFQuestion::entropy_t entropy_t;

    public:
        ListOfRandomRFQuestions(unsigned number_of_questions, boost::random::mt19937& rng)
            : m_questions(), m_rng(rng){
            for(unsigned i = 0; i < number_of_questions; i++)
                m_questions.push_back(_randomPxRatioQuestion());
        }

        // NB: pass by reference of shared pointer - a different pointer might
        // be returned to the one passed in.
        Question_constptr bestQuestionFor(pt_vec const& points,
                                         idx_vec const& pt_indices,
                                         int_vec const& goodness,
                                         cv::Mat const& image,
                                         int& best_terminal_flags,
                                         idx_vec_ptr &test_true,
                                         idx_vec_ptr &test_false) const{
            idx_vec_ptr test_true_temp = boost::make_shared<idx_vec>();
            idx_vec_ptr test_false_temp = boost::make_shared<idx_vec>();
            test_true_temp->reserve(pt_indices.size());
            test_false_temp->reserve(pt_indices.size());
            entropy_t best_entropy = std::numeric_limits<entropy_t>::infinity();
            int terminal;
            qlist_t::const_iterator best = m_questions.end();
            qlist_t::const_iterator i;
            int num_useless_questions = 0;
            for(i = m_questions.begin(); i != m_questions.end(); i++){
                test_true_temp->clear();
                test_false_temp->clear();
                const entropy_t entropy = (*i)->entropyOverTarget(
                    points, pt_indices, goodness, image, terminal,
                    test_true_temp, test_false_temp
                );
                if(entropy < best_entropy){
                    best = i;
                    best_entropy = entropy;
                    best_terminal_flags = terminal;
                    test_true.swap(test_true_temp);
                    test_false.swap(test_false_temp);
                    //debug(5) << "swap" << entropy << "true:" << test_true->size() << "false:" << test_false->size();
                }
                if(entropy == std::numeric_limits<entropy_t>::infinity()){
                    num_useless_questions++;
                }
                if(terminal == TrueGoodFalseBad ||
                   terminal == TrueBadFalseGood){
                    debug(9) << "terminal(" << terminal << ") entropy=" << entropy
                             << "best=" << best_entropy
                             << "test_true:" << test_true->size()
                             << "test_false:" << test_false->size();
                    assert(entropy <= best_entropy);
                    break;
                }else if(terminal){
                    if(entropy == best_entropy)
                        debug(9) << "one-sided terminal(" << terminal << ") entropy=" << entropy
                                 << "best=" << best_entropy
                                 << "test_true:" << test_true->size()
                                 << "test_false:" << test_false->size();
                    else
                        debug(9) << "one-sided terminal(" << terminal << ") entropy=" << entropy
                                 << "best=" << best_entropy
                                 << "test_true:" << test_true_temp->size()
                                 << "test_false:" << test_false_temp->size();
                }
            }
            debug() << 100*float(num_useless_questions) / m_questions.size() << "% useless questions";
            if(best == m_questions.end()){
                debug() << BashColour::Brown
                        << "Tree cannot not distinguish between"
                        << pt_indices.size() << "/" << points.size()
                        << "points";
                test_true.swap(test_true_temp);
                test_false.swap(test_false_temp);
                // if we can't distinguish, presume points are bad:
                best_terminal_flags = test_true->size()? TrueBadFalseGood : TrueGoodFalseBad;
                return m_questions.back();
            }
            return *best;
        }

    private:
        boost::shared_ptr<PxRatioQuestion> _randomPxRatioQuestion(){
            const int Max_Px_Offset = 5;
            boost::random::uniform_real_distribution<float> ratio_dist(1.0, 256);
            boost::random::uniform_int_distribution<int> pxoffset_dist(-Max_Px_Offset,Max_Px_Offset);
            const float ratio = ratio_dist(m_rng);
            const cv::Point a(pxoffset_dist(m_rng), pxoffset_dist(m_rng));
            const cv::Point b(pxoffset_dist(m_rng), pxoffset_dist(m_rng));
            debug(8) << "PxRatioQuestion: (" << a.x <<","<< a.y << "), ("
                     << b.x <<"," << b.y << "), ratio=" << ratio;
            return boost::make_shared<PxRatioQuestion>(a, b, ratio);
        }

        qlist_t m_questions;
        boost::random::mt19937 &m_rng;
};


class Forest{
    private:
        typedef std::vector<cauv::KeyPoint> kp_vec;
    
    public:
        Forest(int max_num_trees)
            : m_trees(), m_max_num_trees(max_num_trees), m_rng(){
        }

        void setMaxSize(int max_num_trees){
            m_max_num_trees = max_num_trees;
            // !!! TODO: remove a random set of items if we need to reduce the
            // number of trees
        }

        std::size_t size() const{
            return m_trees.size();
        }
        
        void addTree(TreeNode_ptr p){
            if(m_trees.size() >= std::size_t(m_max_num_trees)){
                boost::random::uniform_int_distribution<int> remove_idx_dist(0,m_trees.size()-2);
                std::swap(m_trees.back(), m_trees[remove_idx_dist(m_rng)]);
                m_trees.pop_back();
            }
            m_trees.push_back(p);
            info() << "addTree:" << size() << "trees";
        }
        
        bool test(cv::Point const& pt, cv::Mat const& image) const{
            unsigned good = 0;
            foreach(TreeNode_ptr t, m_trees)
                good += t->test(pt, image);
            return good > m_trees.size()/2;
        }

        inline kp_vec filter(kp_vec const& in_kps, cv::Mat const& image) const{
            info() << "filter: " << in_kps.size() << "keypoints...";
            if(!m_trees.size()){
                debug() << "filter: no forest!";
                return in_kps;
            }
            kp_vec r;
            r.reserve(in_kps.size());
            foreach(KeyPoint const& k, in_kps)
                if(test(cv::Point(int(k.pt.x), int(k.pt.y)), image))
                    r.push_back(k);
            info() << "filter: " << r.size() << "passed";
            return r;
        }

        //inline std::vector<cauv::KeyPoint> extract(cv::Mat const& image) const{
        //    
        //}

    private:
        std::vector<TreeNode_ptr> m_trees;
        int m_max_num_trees;
        boost::random::mt19937 m_rng;        
};

// - LearnedKeyPointsNode

/* 
 */
class LearnedKeyPointsNode: public Node{
    private:
        typedef std::vector<cauv::KeyPoint> kp_vec;

    public:
        LearnedKeyPointsNode(ConstructArgs const& args)
            : Node(args),
              m_forest(100),
              m_number_of_questions(10)
        {
        }

        virtual void init(){
            m_speed = fast;
            
            // source image input:
            registerInputID("image");
            registerParamID("bootstrap keypoints", kp_vec());

            // training inputs:
            // These must be set from the same node
            registerParamID("training: keypoints", kp_vec(), "keypoints to update training data", May_Be_Old);
            registerInputID("training: keypoints image", Optional);
            registerParamID("training: goodness", int_vec(), "score values to update training data", May_Be_Old);

            // outputs
            registerOutputID("keypoints", kp_vec()); // all of them
            registerOutputID("image");
            registerOutputID("good keypoints", kp_vec()); // ones that passed the classifier in its current state
            
            // parameters:
            registerParamID("questions", int(200), "number of questions to use for each tree in the classifier forest");
            registerParamID("trees", int(100), "maximum number of trees in the forest");
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

        struct FilterKeyPoints: boost::static_visitor< kp_vec >{
            FilterKeyPoints(Forest const& forest, kp_vec const& kps)
                : m_forest(forest), m_kps(kps){
            }
            kp_vec operator()(cv::Mat a) const{ return m_forest.filter(m_kps, a); }
            kp_vec operator()(NonUniformPolarMat a) const{ return m_forest.filter(m_kps, a.mat); }
            kp_vec operator()(PyramidMat) const{
                error () << __FILE__ << ":" << __LINE__ << "not implemented yet";
                return kp_vec();
            }
            private:
                Forest const& m_forest;
                kp_vec const& m_kps;
        };

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];

            kp_vec training_keypoints = param< kp_vec >("training: keypoints");
            int_vec training_goodness = param< int_vec >("training: goodness");
            image_ptr_t training_img = inputs["training: keypoints image"];

            m_forest.setMaxSize(param<int>("trees"));
            int num_questions = param<int>("questions");
            if(num_questions != m_number_of_questions){
                if(num_questions < 1){
                    error() << "cannot set questions < 1";
                    num_questions = 1;
                }
                if(m_forest.size())
                    warning() << "Changing number of questions after training has started";
                m_number_of_questions = num_questions;
            }
            
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
                const kp_vec bootstrap_keypoints = param< kp_vec >("bootstrap keypoints");
                r["keypoints"] = ParamValue(bootstrap_keypoints);
                augmented_mat_t m = img->augmentedMat();
                r["good keypoints"] = ParamValue(boost::apply_visitor(FilterKeyPoints(m_forest, bootstrap_keypoints), m));
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
                    int terminal;
                    idx_vec_ptr test_true = boost::make_shared<idx_vec>();
                    idx_vec_ptr test_false = boost::make_shared<idx_vec>();
                    test_true->reserve(pt_indices.size());
                    test_false->reserve(pt_indices.size());
                    Question_constptr best_question = m_questions.bestQuestionFor(
                        points, pt_indices, goodness, m, terminal, test_true, test_false
                    );

                    TreeNode_ptr r = boost::make_shared<TreeNode>(
                        boost::cref(best_question), terminal
                    );
                    debug(7) << "Tree Node: true=" << test_true->size()
                             << "false=" << test_false->size()
                             << "terminal=" << (int)terminal;
                    if(!(terminal & TerminalTrue))
                        r->setTrueSide(_trainTree(m, points, *test_true, goodness));                    
                    if(!(terminal & TerminalFalse))
                        r->setFalseSide(_trainTree(m, points, *test_false, goodness));
                    return r;
                }

                pt_vec const& m_points;
                int_vec const& m_point_goodness;
                ListOfRandomRFQuestions const& m_questions;
        };

        void _train(kp_vec const& keypoints, int_vec const& goodness, image_ptr_t img){
            info() << keypoints.size() <<  "kps, "
                   << goodness.size() << "values for training";
            
            pt_vec kps_as_points;
            kps_as_points.reserve(keypoints.size());
            foreach(KeyPoint const& kp, keypoints)
                kps_as_points.push_back(cv::Point(int(kp.pt.x), int(kp.pt.y)));
            
            augmented_mat_t m = img->augmentedMat();
            
            ListOfRandomRFQuestions questions(m_number_of_questions, m_rng);
            TreeNode_ptr new_tree = boost::apply_visitor(
                TreeGrowingVisitor(kps_as_points, goodness, questions), m
            );
            
            m_forest.addTree(new_tree);
        }

    private:
        Forest m_forest;

        int m_number_of_questions;
        boost::random::mt19937 m_rng;

    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef LEARNEDKEYPOINTS_NODE_H
