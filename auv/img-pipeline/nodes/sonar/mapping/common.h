/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_SONAR_SLAM_COMMON_H__
#define __CAUV_SONAR_SLAM_COMMON_H__

#include <Eigen/Dense>

#include <vector>
#include <fstream>

#include <boost/shared_ptr.hpp>

namespace cauv{
namespace imgproc{

// - Forward Declarations
class SlamCloudLocation;
template<typename PointT> class SlamCloudPart;
template<typename PointT> class SlamCloudGraph;
struct IncrementalPose;
struct RelativePose;
struct RelativePoseConstraint;

class GraphOptimiser;

template<typename PointT> class PairwiseMatcher;


// - Typedefs
typedef float descriptor_t;
typedef float score_t;

typedef boost::shared_ptr<SlamCloudLocation> location_ptr;
typedef boost::shared_ptr<RelativePoseConstraint> pose_constraint_ptr;

typedef std::vector<location_ptr> location_vec;
typedef std::vector<pose_constraint_ptr> constraint_vec;


// will find a home for this later
inline static void saveMat(std::ofstream& f, Eigen::Matrix4f const& m){
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            f.write((char*)&m(i,j), sizeof(m(i,j)));
}

inline static void loadMat(std::ifstream& f, Eigen::Matrix4f& m){
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            f.read((char*)&m(i,j), sizeof(m(i,j)));
}

} // namespace imgproc
} // namespace cauv

#endif // ndef __CAUV_SONAR_SLAM_COMMON_H__
