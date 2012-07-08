/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __KMEANS_POINTS_NODE_H__
#define __KMEANS_POINTS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/version.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#if BOOST_VERSION >= 104700
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#else // old boost random hacks
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#warning !! please update your boost version to 1.47 or greater, this is way too hacky !!
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include <boost/random/mersenne_twister.hpp>
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

#include <opencv2/core/core.hpp>

#include <utility/foreach.h>
#include "../node.h"


namespace cauv{
namespace imgproc{

class KMeansPointsNode: public Node{
    public:
        KMeansPointsNode(ConstructArgs const& args) :
                Node(args)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerParamID< std::vector<KeyPoint> >("keypoints", std::vector<KeyPoint>(), "", Must_Be_New);

            // two outputs:
            registerOutputID("clusters", std::vector<Ellipse>());

            // parameters:
            //   K: the number of clusters
            registerParamID<int>("K", 5);
            registerParamID<int>("max iters", 50);
            registerParamID<int>("search iters", 5);
            registerParamID<float>("cluster size penalty", 0.1);
        }

    protected:
        struct NormalDist
        {
            NormalDist(){ }

            NormalDist(float size_penalty)
                : mean(Eigen::Vector2f::Zero()),
                  covar(Eigen::Matrix2f::Zero()),
                  covar_inv(Eigen::Matrix2f::Zero()),
                  sum(Eigen::Vector2f::Zero()),
                  sumsquared(Eigen::Matrix2f::Zero()),
                  numpoints(0),
                  size_penalty(size_penalty),
                  calculated_penalty(0){
            }

            float logP(KeyPoint const& kp) const{
                // call bake() first!
                const Eigen::Vector2f q = Eigen::Vector2f(kp.pt.x, kp.pt.y) - mean;
                const Eigen::RowVector2f qt_cvi = q.transpose() * covar_inv;
                return (-0.5 * float(qt_cvi * q)) - calculated_penalty;
            }

            void add(KeyPoint const& kp){
                numpoints++;
                Eigen::Vector2f p(kp.pt.x, kp.pt.y);
                sum += p;
                sumsquared += p * p.transpose();
            }
            
            bool bake(){
                // sets the mean and covar_inv from the sample sum and sumsquared
                if(numpoints > 2){
                    mean = sum / numpoints;
                    covar = (sumsquared - 2 * (sum * mean.transpose())) / numpoints + mean * mean.transpose();
                    // if the points are colinear, spread things out a bit:
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covar);
                    const Eigen::Vector2f eigenvalues = solver.eigenvalues();
                    if(eigenvalues[0] < 1e-3 * eigenvalues[1]){
                        debug(2) << "adjusting eigenvalues!";
                        Eigen::Matrix2f l = solver.eigenvalues().asDiagonal();
                        Eigen::Matrix2f q = solver.eigenvectors();
                        // set minimum smallest eigenvalue:
                        l(0,0) = l(1,1) * 1e-3;
                        covar = q * l * q.transpose();
                    }
                    debug(7) << "mean:" << mean.transpose()
                             << "covariance:\n"
                             << covar(0,0) << covar(0,1) << '\n' << covar(1,0) << covar(1,1);
                    calculated_penalty = std::max(std::fabs(eigenvalues[0]), std::fabs(eigenvalues[1]));
                }else if(numpoints != 0){
                    mean = sum / numpoints;
                    covar = Eigen::Matrix2f::Identity() * 10;
                }else{
                    return false;
                }
                covar_inv = covar.inverse();
                return true;
            }

            Ellipse ellipse() const{
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covar);
                const float minor = std::sqrt(solver.eigenvalues()[0]);
                const float major = std::sqrt(solver.eigenvalues()[1]);
                const Eigen::Vector2f major_axis = solver.eigenvectors().col(1);
                const float angle = std::atan2(major_axis[1], major_axis[0]);
                debug(2) << "Ellipse: major=" << major << "minor=" << minor << "angle=" << angle;
                return Ellipse(
                    floatXY(mean[0], mean[1]),
                    minor*2.5, // 95%
                    major*2.5, // 95%
                    angle
                );
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            Eigen::Vector2f mean;
            Eigen::Matrix2f covar;
            Eigen::Matrix2f covar_inv;

            Eigen::Vector2f sum;
            Eigen::Matrix2f sumsquared;
            uint32_t numpoints;
            float size_penalty;
            float calculated_penalty;
        };

        void doWork(in_image_map_t&, out_map_t& r){
            const int k = param<int>("K");
            const int max_iters = param<int>("max iters");
            const int search_iters = param<int>("search iters");
            const float size_penalty = param<float>("cluster size penalty");
            
            // In a rather special case, we want to propagate the uid from the
            // input keypoints to the output ellipses, NOT have the out_map_t
            // autmagially assign a uid based on the input (since we don't have
            // any 'input' images)
            std::vector<KeyPoint> keypoints;
            UID kps_uid;
            boost::tie(keypoints, kps_uid) = paramAndUID< std::vector<KeyPoint> >("keypoints");

            if(!keypoints.size())
                return;

            boost::random::uniform_int_distribution<int> random_point_dist(0,keypoints.size()-1);
            
            std::vector<NormalDist, Eigen::aligned_allocator<NormalDist> > best_search_clusters(k, NormalDist());
            float best_search_log_p = -std::numeric_limits<float>::max();

            for(int search = 0; search < search_iters; search++){
                std::vector<int> assignments(keypoints.size(), -2);
                float search_log_p = 0;
                std::vector<NormalDist, Eigen::aligned_allocator<NormalDist> > clusters(k, NormalDist());
                for(int iter = 0; iter < max_iters; iter++){
                    // estimate cluster distributions
                    foreach(NormalDist& c, clusters)
                        c = NormalDist(size_penalty);
                    if(iter == 0){
                        // start with means assigned to random points, large variances:
                        foreach(NormalDist& c, clusters){
                            KeyPoint kp = keypoints[random_point_dist(gen)];
                            c.mean[0] = kp.pt.x;
                            c.mean[1] = kp.pt.y;
                            c.covar = Eigen::Matrix2f::Identity() * 100 / (1.0 + 100*size_penalty);
                            c.covar_inv = c.covar.inverse();
                        }
                    }else{
                        for(size_t i = 0; i < keypoints.size(); i++)
                            clusters[assignments[i]].add(keypoints[i]);
                        foreach(NormalDist& c, clusters){
                            bool baked_successfully = c.bake();
                            if(!baked_successfully){
                                // no points: set to random position
                                KeyPoint kp = keypoints[random_point_dist(gen)];
                                c.mean[0] = kp.pt.x;
                                c.mean[1] = kp.pt.y;
                                c.covar = Eigen::Matrix2f::Identity() * 20;
                                c.covar_inv = c.covar.inverse();
                            }
                        }
                    }

                    // update assignments
                    bool assignments_changed = false;
                    for(size_t i = 0; i < keypoints.size(); i++){
                        int best_assignment = -1;
                        float best_log_p = -std::numeric_limits<float>::max();
                        for(int j = 0; j < k; j++){
                            const float p = clusters[j].logP(keypoints[i]);
                            if(p >= best_log_p){
                                best_log_p = p;
                                best_assignment = j;
                            }
                        }
                        search_log_p += best_log_p;
                        if(assignments[i] != best_assignment){
                            assignments_changed = true;
                            assignments[i] = best_assignment;
                        }
                    }
                    if(!assignments_changed){
                        debug(2) << "break iter" << iter;
                        break;
                    }
                }

                if(search_log_p > best_search_log_p){
                    best_search_log_p = search_log_p;
                    best_search_clusters = clusters;
                }
            }

            std::vector<Ellipse> ret;
            foreach(NormalDist const& c, best_search_clusters)
                ret.push_back(c.ellipse());

            // !!! TODO: should use InternalParamValue assignment to preserve UID
            r["clusters"] = ret;
            
            r.internalValue("clusters") = InternalParamValue(
                ret, kps_uid
            );
        }

    private:
        boost::random::mt19937 gen;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
