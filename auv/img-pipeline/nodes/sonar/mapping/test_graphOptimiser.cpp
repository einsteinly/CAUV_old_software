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

    //assert_very_close(*a, ci::SlamCloudLocation(-0.25,0,0));
    //assert_very_close(*b, ci::SlamCloudLocation(1.25,0,0));
}

void testClassicLoopClose(){
    ci::GraphOptimiserV1 g;
    ci::constraint_vec constraints;
    ci::constraint_vec constraints2;


    typedef ci::IncrementalPoseConstraint IncrPC;
    typedef ci::IncrementalPose IncrP;

    // with angles:
    ci::location_ptr a = boost::make_shared<ci::SlamCloudLocation>(0,0,0); 
    ci::location_ptr b = boost::make_shared<ci::SlamCloudLocation>(1,0, M_PI * 2.2/2);
    ci::location_ptr c = boost::make_shared<ci::SlamCloudLocation>(1,2, M_PI * 3.2/2);
    ci::location_ptr d = boost::make_shared<ci::SlamCloudLocation>(0,2, M_PI * 4.2/2);

    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 1, 0, M_PI * 2.2/2), a, b));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0, 2, M_PI), b, c));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP(-1, 0, M_PI), c, d));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0,-1, M_PI), d, a)); 

    // no angles:
    ci::location_ptr a2 = boost::make_shared<ci::SlamCloudLocation>(0,0,0); 
    ci::location_ptr b2 = boost::make_shared<ci::SlamCloudLocation>(1,0,0);
    ci::location_ptr c2 = boost::make_shared<ci::SlamCloudLocation>(1,2,0);
    ci::location_ptr d2 = boost::make_shared<ci::SlamCloudLocation>(0,2,0);

    constraints2.push_back(boost::make_shared<IncrPC>(IncrP( 1, 0, 0), a2, b2));
    constraints2.push_back(boost::make_shared<IncrPC>(IncrP( 0, 2, 0), b2, c2));
    constraints2.push_back(boost::make_shared<IncrPC>(IncrP(-1, 0, 0), c2, d2));
    constraints2.push_back(boost::make_shared<IncrPC>(IncrP( 0,-1, 0), d2, a2)); 

    /*
    ci::location_ptr a = boost::make_shared<ci::SlamCloudLocation>(0,0,0);

    ci::location_ptr b = boost::make_shared<ci::SlamCloudLocation>(1,-0.1,0);
    ci::location_ptr c = boost::make_shared<ci::SlamCloudLocation>(2,-0.2,0);
    ci::location_ptr d = boost::make_shared<ci::SlamCloudLocation>(3,-0.3,0);

    ci::location_ptr e = boost::make_shared<ci::SlamCloudLocation>(3,0.7,0);
    ci::location_ptr f = boost::make_shared<ci::SlamCloudLocation>(3,2.6,0);
    ci::location_ptr g = boost::make_shared<ci::SlamCloudLocation>(3,2.5,0);

    ci::location_ptr h = boost::make_shared<ci::SlamCloudLocation>(2,2.4,0);
    ci::location_ptr i = boost::make_shared<ci::SlamCloudLocation>(1,2.3,0);
    ci::location_ptr j = boost::make_shared<ci::SlamCloudLocation>(0,2.2,0);

    ci::location_ptr k = boost::make_shared<ci::SlamCloudLocation>(0,2.0,0);
    ci::location_ptr l = boost::make_shared<ci::SlamCloudLocation>(0,1.0,0);

    typedef ci::IncrementalPoseConstraint IncrPC;
    typedef ci::IncrementalPose IncrP;

    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 1, -0.1, 0), a, b));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 1, -0.1, 0), b, c));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 1, -0.1, 0), c, d));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0,    1, 0), d, e));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0,  0.9, 0), e, f));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0,  0.9, 0), f, g));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP(-1, -0.1, 0), g, h));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP(-1, -0.1, 0), h, i));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP(-1, -0.1, 0), i, j));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0, -1.2, 0), j, k));
    constraints.push_back(boost::make_shared<IncrPC>(IncrP( 0,  1.0, 0), k, l));
    */

    g.optimiseGraph(constraints);
    
    std::cout << "Test Classic Loop Close (1):" << std::endl;
    std::cout << a << std::endl;
    std::cout << b << std::endl;
    std::cout << c << std::endl;
    std::cout << d << std::endl;

    
    g.optimiseGraph(constraints2);
    
    std::cout << "Test Classic Loop Close (no angles) (1):" << std::endl;
    std::cout << a2 << std::endl;
    std::cout << b2 << std::endl;
    std::cout << c2 << std::endl;
    std::cout << d2 << std::endl;
    
}

int main(){
    debug::setLevel(0);

    testConsistency();
    testLoopEvenness();
    testClassicLoopClose();
}

