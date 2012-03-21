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

#include <common/cauv_utils.h>
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
            registerOutputID("clusters", std::vector<KeyPoint>());

            // parameters:
            //   K: the number of clusters
            registerParamID<int>("K", 5);
        }

    protected:
        struct NormalDist
        {
            NormalDist()
                : mean(Eigen::Vector2f::Zero()),
                  covar(Eigen::Matrix2f::Zero()),
                  covar_inv(Eigen::Matrix2f::Zero()),
                  sum(Eigen::Vector2f::Zero()),
                  sumsquared(Eigen::Matrix2f::Zero()){
            }

            float logP(KeyPoint const& kp) const{
                // call bake() first!
                const Eigen::Vector2f q = Eigen::Vector2f(kp.pt.x, kp.pt.y) - mean;
                const Eigen::RowVector2f qt_cvi = q.transpose() * covar_inv;
                return -0.5 * float(qt_cvi * q);
            }

            void add(KeyPoint const& kp){
                numpoints++;
                Eigen::Vector2f p(kp.pt.x, kp.pt.y);
                sum += p;
                sumsquared += p * p.transpose();
            }

            void bake(){
                // sets the mean and covar_inv from the sample sum and sumsquared
                if(numpoints > 2){
                    mean = sum / numpoints;
                    covar = (sumsquared - 2 * (sum * mean.transpose())) / numpoints + mean * mean.transpose();
                    // if the points are colinear, spread things out a bit:
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covar);
                    if(solver.eigenvalues()[0] < 1e-3 * solver.eigenvalues()[1]){
                        debug() << "adjusting eigenvalues!";
                        Eigen::Matrix2f l = solver.eigenvalues().asDiagonal();
                        Eigen::Matrix2f q = solver.eigenvectors();
                        // set minimum smallest eigenvalue:
                        l(0,0) = l(1,1) * 1e-3;
                        covar = q * l * q.transpose();
                    }
                    covar_inv = covar.inverse();
                }else if(numpoints == 2){
                    mean = sum / numpoints;
                    covar_inv = Eigen::Matrix2f::Identity();
                }else if(numpoints != 0){
                    mean = sum / numpoints;
                    covar_inv = Eigen::Matrix2f::Zero();
                }else{
                    mean = Eigen::Vector2f::Zero();
                    covar_inv = Eigen::Matrix2f::Zero();
                }
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            Eigen::Vector2f mean;
            Eigen::Matrix2f covar;
            Eigen::Matrix2f covar_inv;

            Eigen::Vector2f sum;
            Eigen::Matrix2f sumsquared;
            uint32_t numpoints;
        };

        void doWork(in_image_map_t&, out_map_t& r){
            const int k = param<int>("K");
            const std::vector<KeyPoint> keypoints = param<std::vector<KeyPoint> >("keypoints");

            boost::random::uniform_int_distribution<int> assign_dist(0,k-1);

            // start with random assignments:
            std::vector<int> assignments(keypoints.size());
            foreach(int& a, assignments)
                a = assign_dist(gen);

            std::vector<NormalDist, Eigen::aligned_allocator<NormalDist> > clusters(k);
            for(int iter = 0; iter < 100; iter++){
                // estimate cluster distributions
                foreach(NormalDist& c, clusters)
                    c = NormalDist();
                for(size_t i = 0; i < keypoints.size(); i++)
                    clusters[assignments[i]].add(keypoints[i]);
                foreach(NormalDist& c, clusters)
                    c.bake();

                // update assignments
                bool assignments_changed = false;
                for(size_t i = 0; i < keypoints.size(); i++){
                    int best_assignment = -1;
                    float best_log_p = std::numeric_limits<float>::min();
                    for(int j = 0; j < k; j++){
                        const float p = clusters[j].logP(keypoints[i]);
                        if(p > best_log_p){
                            best_log_p = p;
                            best_assignment = j;
                        }
                    }
                    if(assignments[i] != best_assignment){
                        assignments_changed = true;
                        assignments[i] = best_assignment;
                    }
                }
                if(!assignments_changed)
                    break;
            }

            // also TODO: angle / something more descriptive
            std::vector<KeyPoint> ret;
            foreach(NormalDist const& c, clusters)
                ret.push_back(
                    KeyPoint(floatXY(c.mean[0], c.mean[1]),
                    std::sqrt(c.covar.trace()),
                    0,
                    0,
                    0,
                    0
                ));

            // !!! TODO: should use InternalParamValue assignment to preserve UID
            r["clusters"] = ret;
        }

    private:
        boost::random::mt19937 gen;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
