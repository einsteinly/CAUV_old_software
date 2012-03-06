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

#include <vector>

#include <boost/shared_ptr.hpp>

namespace cauv{
namespace imgproc{

// - Forward Declarations
class SlamCloudLocation;
template<typename PointT> class SlamCloudPart;
template<typename PointT> class SlamCloudGraph;
struct IncrementalPose;
struct IncrementalPoseConstraint;

class GraphOptimiser;

template<typename PointT> class PairwiseMatcher;


// - Typedefs
typedef float descriptor_t;
typedef float score_t;

typedef boost::shared_ptr<SlamCloudLocation> location_ptr;
typedef boost::shared_ptr<IncrementalPoseConstraint> pose_constraint_ptr;

typedef std::vector<location_ptr> location_vec;
typedef std::vector<pose_constraint_ptr> constraint_vec;


} // namespace imgproc
} // namespace cauv

#endif // ndef __CAUV_SONAR_SLAM_COMMON_H__
