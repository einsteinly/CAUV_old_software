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

#include <iostream>
#include <ostream>

#include <boost/make_shared.hpp>

#include "graphOptimiser.h"
#include "slamCloud.h"

namespace ci = cauv::imgproc;

std::ostream& operator<< (std::ostream& os, ci::location_ptr const& p){
    if(p){
        const Eigen::Matrix2f rot = p->globalTransform().block<2,2>(0, 0);
        const Eigen::Vector2f   t = p->globalTransform().block<2,1>(0, 3);
        const Eigen::Vector2f tmp = rot * Eigen::Vector2f(1,0); 
        const float rdeg = (180/M_PI) * std::atan2(tmp[1], tmp[0]);
        return os << "loc{"<< t[0] << ", " << t[1] << ": " << rdeg <<" deg}";
    }else{
        return os << "loc{NULL}";
    }
}

void assert_very_close(ci::SlamCloudLocation const& a, ci::SlamCloudLocation const& b){
    assert((a.globalTransform() - b.globalTransform()).squaredNorm() < 1e-6);
}

void testConsistency(){
    ci::GraphOptimiserV1 g;
    ci::constraint_vec constraints;
     
    ci::location_ptr a = boost::make_shared<ci::SlamCloudLocation>(0,0,0);
    ci::location_ptr b = boost::make_shared<ci::SlamCloudLocation>(1,0,0);

    constraints.push_back(boost::make_shared<ci::IncrementalPoseConstraint>(
        // a_to_b is +1, 0
        ci::IncrementalPose(1, 0, 0), a, b
    ));
    constraints.push_back(boost::make_shared<ci::IncrementalPoseConstraint>(
        // b to a is -1, 0
        ci::IncrementalPose(-1, 0, 0), b, a
    ));

    g.optimiseGraph(constraints);
    
    std::cout << "Test Consistency:" << std::endl;
    std::cout << a << std::endl;
    std::cout << b << std::endl;

    assert_very_close(*a, ci::SlamCloudLocation(0,0,0));
    assert_very_close(*b, ci::SlamCloudLocation(1,0,0));
}

void testLoopEvenness(){
    ci::GraphOptimiserV1 g;
    ci::constraint_vec constraints;
     
    ci::location_ptr a = boost::make_shared<ci::SlamCloudLocation>(0,0,0);
    ci::location_ptr b = boost::make_shared<ci::SlamCloudLocation>(1,0,0);

    constraints.push_back(boost::make_shared<ci::IncrementalPoseConstraint>(
        // a_to_b is +1, 0
        ci::IncrementalPose(1, 0, 0), a, b
    ));
    constraints.push_back(boost::make_shared<ci::IncrementalPoseConstraint>(
        // b to a is twice as big - we should end up half way inbetween!
        ci::IncrementalPose(-2, 0, 0), b, a
    ));

    g.optimiseGraph(constraints);
    
    std::cout << "Test Loop Evenness:" << std::endl;
    std::cout << a << std::endl;
    std::cout << b << std::endl;

    assert_very_close(*a, ci::SlamCloudLocation(-0.25,0,0));
    assert_very_close(*b, ci::SlamCloudLocation(1.25,0,0));
}

int main(){
    debug::setLevel(10);

    testConsistency();
    testLoopEvenness();
}

