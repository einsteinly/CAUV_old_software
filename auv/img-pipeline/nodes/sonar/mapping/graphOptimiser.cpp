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

#include "graphOptimiser.h"

#include <map>
#include <vector>
#include <cmath>
#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <utility/foreach.h>
#include <utility/streamops.h>
#include <utility/string.h>

#include <debug/cauv_debug.h>

#include "slamCloud.h"

using namespace cauv;
using namespace cauv::imgproc;

IncrementalPose IncrementalPose::from4dAffine(Eigen::Matrix4f const& a){
    const Eigen::Matrix2f r = a.block<2,2>(0, 0);
    const Eigen::Vector2f t = a.block<2,1>(0, 3);

    const Eigen::Vector2f tmp = r*Eigen::Vector2f(1,0);
    const float rz = std::atan2(tmp[1], tmp[0]);
    return IncrementalPose(Eigen::Vector3f(t[0], t[1], rz));
}

IncrementalPose IncrementalPose::from4dAffineDiff(
    Eigen::Matrix4f const& from,
    Eigen::Matrix4f const& to){
    return from4dAffine(to) -= from4dAffine(from);
}


class Node;
class Arc;
typedef boost::shared_ptr<Node> Node_ptr;
typedef boost::shared_ptr<Arc> Arc_ptr;


struct Arc{
    Node_ptr from;
    Node_ptr to;
    IncrementalPose x; // (to - from)

    // IncrementalPose has Eigen::Vector3f
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Node{
    Node() : p(), parent_link(), child_links(), tag(-1){ }
    Node(location_ptr const& p,
         Arc_ptr const& parent_link,
         Arc_ptr const& child = Arc_ptr(),
         int const& tag = -1)
        : p(p), parent_link(parent_link), child_links(), tag(tag){
        if(child)
            child_links.push_back(child);
    }

    location_ptr p;
    Arc_ptr parent_link;
    std::vector<Arc_ptr> child_links;
    // tag is used to store the ID of the disjoint set during construction of
    // the tree, then the distance of the node from the root once the tree has
    // been constructed
    int tag;
};

std::ostream& operator<<(std::ostream& os, IncrementalPose const& a){
    // convert angle to degrees for display
    return os << "IncrPose{dx:"<< a.x[0] << " dy:" << a.x[1] << " dth:"<< (180/M_PI)*a.x[2] << "}";
}
std::ostream& operator<<(std::ostream& os, Arc const& a){
    return os << "Arc{fr:"<< a.from << " to:" << a.to <<"}";
}
std::ostream& operator<<(std::ostream& os, Arc_ptr const& a){
    if(a)
        return os << *a;
    else    
        return os << "Arc_ptr(NULL)";
}
std::ostream& operator<<(std::ostream& os, Node const& n){
    return os << "Node{("<<&n<<")p:"<< n.p <<" pl:"<< n.parent_link <<" kids:"<< n.child_links << " tag:" << n.tag << "}";
}

/* Make new_parent the new parent of child along arc a (which may require
 * child's existing parent to be recursively reparented)
 *
 * pre-conditions: a->from and a->to are set as if the parenting has already
 * taken place, but child->parent_link has not yet been changed from its old
 * value.
 */
static void adoptSubTree(Node_ptr new_parent, Arc_ptr a, Node_ptr child){
    assert(child);
    assert(new_parent);
    assert(a->to == child);
    assert(a->from == new_parent);

    Arc_ptr b = child->parent_link;
    child->parent_link = a;
    if(b){
        // reverse direction of relationship:
        b->x = -b->x;
        assert(b->to == child); 
        std::vector<Arc_ptr>::iterator t = std::remove(b->from->child_links.begin(), b->from->child_links.end(), b);
        assert(t == (b->from->child_links.end()-1));
        b->from->child_links.pop_back();
        b->to = b->from;
        b->from = child;
        child->child_links.push_back(b);

        new_parent = child;
        a = b;
        child = b->to;
        adoptSubTree(new_parent, a, child);
    }
}

/* Tag nodes in the tree with their distance from the root.
 */
static void retagByDepth(int depth, Node_ptr p){
    p->tag = depth;
    foreach(Arc_ptr const& a, p->child_links){
        // use this opportunity to sanity check the tree, too
        assert(a->from == p);
        retagByDepth(depth+1, a->to);
    }
}


/* Build a spanning tree from a set of constraints. Each link in the tree
 * defines the incremental pose (as defined by Grisetti et al) between two
 * nodes (which are poses).
 * The initial values of the incremental pose are calculated from THE CURRENT
 * POSITIONS OF THE NODES, not from the values of the constraints that are used
 * to build the form of the tree: think of the constraints as being used to
 * choose a sensible parametrisation for the problem.
 */
static Node_ptr spanningTree(
    constraint_vec const& constraints,
    std::map<location_ptr, Node_ptr>& node_lookup
){
    assert(constraints.size());
    assert(node_lookup.size() == 0);

    std::map<location_ptr, Node_ptr>::const_iterator i, j;

    int tag = 0;
    std::vector<Node_ptr> set_root;

    Node_ptr root = boost::make_shared<Node>();
    root->parent_link.reset();
    root->p = (*constraints.begin())->b;
    root->tag = tag;
    node_lookup[root->p] = root;

    // set_root[tag] = root node of disjoint set
    set_root.push_back(root);

    int non_spanning = 0;

    debug(5) << "spanning tree construction over" << constraints.size() << "constraints...";
    foreach(pose_constraint_ptr p, constraints){
        i = node_lookup.find(p->a);
        j = node_lookup.find(p->b);
        if(i != node_lookup.end()){
            if(j == node_lookup.end()){
                // add p->b relative to existing p->a
                Arc_ptr a(new Arc);
                a->from = i->second;
                a->from->child_links.push_back(a);
                a->to = boost::make_shared<Node>(p->b, a, Arc_ptr(), a->from->tag);
                a->x = IncrementalPose::from4dAffineDiff(a->from->p->globalTransform(), a->to->p->globalTransform());
                node_lookup[a->to->p] = a->to;
            }else{
                // if a is not connected to b, this arc connects them:
                if(set_root[i->second->tag] != set_root[j->second->tag]){
                    Arc_ptr a(new Arc);
                    if((!i->second->parent_link) || (!j->second->parent_link)){
                        if(i->second->parent_link){
                            a->from = i->second;
                            a->to = j->second;
                            a->to->parent_link = a;
                        }else{
                            a->from = j->second;
                            a->to = i->second;
                            a->to->parent_link = a;
                        }
                        a->from->child_links.push_back(a);
                        debug(7) << "reparent (easy)" << a->to->tag << "to" << a->from->tag << "(" << set_root[a->from->tag] << ")";
                    }else{
                        // both sides of this constraint have parents already -
                        // need to reverse parent/child relations in one of the
                        // trees :(
                        a->from = i->second;
                        a->to = j->second;
                        a->from->child_links.push_back(a);
                        debug(7) << "reparent (hard)" << a->to->tag << "to" << a->from->tag << "(" << set_root[a->from->tag] << ")";
                        // Make i the new parent of tree j->second via arc a
                        adoptSubTree(i->second, a, j->second);
                        assert(a->to->parent_link == a);
                    }
                    a->x = IncrementalPose::from4dAffineDiff(a->from->p->globalTransform(), a->to->p->globalTransform());
                    // careful not to pass a reference to an element in the
                    // vector to the replace algorithm!
                    Node_ptr replace_this = set_root[a->to->tag];
                    std::replace(set_root.begin(), set_root.end(), replace_this, set_root[a->from->tag]);
                }else{
                    // already in the tree
                    non_spanning++;
                }
            }
        }else{
            Arc_ptr a(new Arc);
            if(j != node_lookup.end()){
                // add p->a relative to existing p->b
                a->from = j->second;
                a->from->child_links.push_back(a);
                a->to = boost::make_shared<Node>(p->a, a, Arc_ptr(), a->from->tag);
                node_lookup[a->to->p] = a->to;
            }else{
                // add new disjoint arc
                ++tag;

                a->from = boost::make_shared<Node>(p->a, Arc_ptr(), a, tag);
                a->to   = boost::make_shared<Node>(p->b, a, Arc_ptr(), tag);
                node_lookup[a->from->p] = a->from;
                node_lookup[a->to->p] = a->to;

                assert(set_root.size() == unsigned(tag));
                set_root.push_back(a->from);
            }
            // set incremental pose to current state:
            a->x = IncrementalPose::from4dAffineDiff(a->from->p->globalTransform(), a->to->p->globalTransform());
        }
        foreach(Node_ptr const& np, set_root){
            assert(!(np->parent_link));
        }
    }
    debug(3) << "spanning tree construction complete:"
             << node_lookup.size() << "nodes, "
             << non_spanning << "non-spanning constraints";

    // spanning tree property invariant check:
    if(non_spanning != int(constraints.size() - (node_lookup.size()-1))){
        // uh oh, not a spanning tree. What went wrong?

        std::set<Node_ptr> node_dump;
        std::set<Arc_ptr> arc_dump;
        for(std::map<location_ptr, Node_ptr>::const_iterator i = node_lookup.begin(); i != node_lookup.end(); i++){
            node_dump.insert(i->second);
            if(i->second->parent_link)
                arc_dump.insert(i->second->parent_link);
            foreach(Arc_ptr ap, i->second->child_links)
                arc_dump.insert(ap);
        }
        mkStr s;
        s << "Nodes:\n\t" << node_dump;
        s << "\nArcs:\n\t" << arc_dump;

        error() << "spanning tree failure: not a spanning tree!"
                << "\n\tconstraints:" << constraints.size()
                << "\n\tnon-spaning:" << non_spanning
                << "\n\t   spanning:" << node_lookup.size()
                << "\n\t  set roots:" << set_root
                << "\n\t       tree:" << std::string(s);
        assert(0);
    }

    // re-tag the tree so that tag counts the depth from the root node (this
    // lets common parent nodes be identified easily)
    retagByDepth(0, set_root[0]);
    
    // root node invariant check:
    if(set_root[0]->parent_link){
        // uh oh, not a spanning tree. What went wrong?

        std::set<Node_ptr> node_dump;
        std::set<Arc_ptr> arc_dump;
        for(std::map<location_ptr, Node_ptr>::const_iterator i = node_lookup.begin(); i != node_lookup.end(); i++){
            node_dump.insert(i->second);
            if(i->second->parent_link)
                arc_dump.insert(i->second->parent_link);
            foreach(Arc_ptr ap, i->second->child_links)
                arc_dump.insert(ap);
        }
        mkStr s;
        s << "Nodes:\n\t" << node_dump;
        s << "\nArcs:\n\t" << arc_dump;

        error() << "spanning tree failure: root has a parent!"
                << "\n\tconstraints:" << constraints.size()
                << "\n\tnon-spaning:" << non_spanning
                << "\n\t   spanning:" << node_lookup.size()
                << "\n\t  set roots:" << set_root
                << "\n\t       tree:" << std::string(s);
        assert(0);
    }
    return set_root[0];
}

// Visit nodes and arcs (static typing means arc visits will be optimised out
// if v has no-ops for them). Return the number of nodes visited
template<typename PathVisitor>
static uint32_t visitBetweenNodes(Node_ptr b, Node_ptr a, PathVisitor const& v){
    bool reverse_order = false;
    if(b->tag > a->tag){
        std::swap(a, b);
        reverse_order = true;
    }
    // Generate the complete path before traversing it, path order is from
    // lowest level node to highest level node

    std::vector<Node_ptr> path_end;   // end (a) is at the start of this
    std::vector<Node_ptr> path_start; // start (b) is at the start of this

    // first traverse up from lowest level (highest tag) node to the level of
    // the highest:
    while(a->tag > b->tag){
        path_end.push_back(a);
        a = a->parent_link->from;
    }

    // now move both nodes up until we reach a common parent:
    while(a != b){
        path_end.push_back(a);
        path_start.push_back(b);
        a = a->parent_link->from;
        b = b->parent_link->from;
    }
    
    // now visit it with the supplied visitor: operator() is applied to nodes,
    // ascend() and descend() are applied to arcs
    if(reverse_order){
        foreach(Node_ptr p, path_end){
            v(*p);
            v.ascend(*p->parent_link);
        }
        // visit top-level node (== a == b)
        v(*a);
        reverse_foreach(Node_ptr p, path_start){
            v.descend(*p->parent_link);
            v(*p);
        }
    }else{
        foreach(Node_ptr p, path_start){
            v(*p);
            v.ascend(*p->parent_link);
        }
        // visit top-level node (== a == b)
        v(*a);
        reverse_foreach(Node_ptr p, path_end){
            v.descend(*p->parent_link);
            v(*p);
        }
    }

    return 1 + path_start.size() + path_end.size();
}

// visits every node, and every arc in each direction, 
template<typename NodeVisitor>
static void visitDepthFirst(Node_ptr root, NodeVisitor const& v){
    v(*root);
    foreach(Arc_ptr const& a, root->child_links){
        v.descend(*a);
        visitDepthFirst(a->to, v);
        v.ascend(*a);
    }
}

struct MaxLevel{
    MaxLevel(int& result)
        : max_level(result){
    }

    void operator()(Node const& node) const{
        if(node.tag > max_level){
            max_level = node.tag;
        }
    }
    void ascend(Arc const&) const { }
    void descend(Arc const&) const { }

    int& max_level;
};

// for dev:
struct PrintNode{
    PrintNode()
        : m_count(0), m_debug_stream(){
        m_debug_stream << "Path{levels:";
    }
    ~PrintNode(){
        m_debug_stream << /*"\n" <<*/ ": "<< m_count << "nodes total}";
    }
    void operator()(Node const& node) const{
        //m_debug_stream << "\n\t" << node;
        m_debug_stream << " " << node.tag << " ";
        m_count++;
    }
    void ascend(Arc const&) const { m_debug_stream << "u"; }
    void descend(Arc const&) const { m_debug_stream << "d"; }

    mutable int m_count;
    mutable debug m_debug_stream;
};

struct AccumulatePose{
    AccumulatePose(IncrementalPose& p)
        : m_pose(p){
    }

    void operator()(Node const&) const{ }

    void ascend(Arc const& arc) const {
        m_pose -= arc.x;
    }
    void descend(Arc const& arc) const {
        m_pose += arc.x;
    }
    
    IncrementalPose& m_pose;
};

// this one actually modifies the tree: add p to descending arcs traversed,
// subtract it from ascending ones
struct AddPose{
    AddPose(IncrementalPose const& p)
        : m_pose_to_add(p){
    }

    void operator()(Node const&) const{ }

    void ascend(Arc& arc) const {
        // subtract on the way up
        arc.x -= m_pose_to_add;
    }
    void descend(Arc& arc) const {
        // add on the way down
        arc.x += m_pose_to_add;
    }

    IncrementalPose const& m_pose_to_add;
};

// only apply this to top-down visits (ie, root first!)
struct ApplyPose{
    void operator()(Node const& node) const{
        assert(!node.p->relativeTo());
        if(node.parent_link){
            node.p->setRelativeTransform(
                node.parent_link->x.applyTo(node.parent_link->from->p->relativeTransform())
            );
        }else{
            // tree root stays in the same place!
            // !!! TODO: the tree root is a pretty arbitrary node - really the
            // oldest one should stay in the same place, or something
        }
    }

    void ascend(Arc const&) const { }
    void descend(Arc const&) const { }
};

static bool poseConstraintTagLess(
    pose_constraint_ptr const& l,
    pose_constraint_ptr const& r
){
    return l->tag < r->tag;
}

/* Optimise a constraint graph.
 * !!! paper ref here (Olson et al, Grisetti et al)
 * !!! this could be provided by an external class like scan matching
 *     so that methods can be easily compared.
 */
void GraphOptimiserV1::optimiseGraph(
    constraint_vec& constraints,
    constraint_vec const& /*new_constraints*/ // currently ignored, could be used to choose tree parametrisation, or more intrusively
) const{
    std::map<location_ptr, Node_ptr> node_lookup;
    std::map<location_ptr, Node_ptr>::const_iterator ait, bit;

    Node_ptr root = spanningTree(constraints, node_lookup);

    // Sort the constraints into the best order for processing: to do this,
    // first determine the 'level' of each constraint - the node nearest the
    // root in the path between the two ends of the constraint in the spanning
    // tree
    foreach(pose_constraint_ptr p, constraints){
        p->tag = 0;
        //p->weight = 1.0; DONT reset weights: let them be persistent :)
        ait = node_lookup.find(p->a); assert(ait != node_lookup.end());
        bit = node_lookup.find(p->b); assert(bit != node_lookup.end());
        visitBetweenNodes(ait->second, bit->second, MaxLevel(p->tag));
    }

    std::sort(constraints.begin(), constraints.end(), poseConstraintTagLess);
    
    float lambda = 1.0f;
    for(int iter = 0; iter < m_max_iters; iter++){
        debug() << "SGD iter" << iter;
        
        int ignored_constraints = 0;
        float sum_weighted_error = 0;
        foreach(pose_constraint_ptr p, constraints){
            debug(3) << "process constraint: level" << p->tag;
            ait = node_lookup.find(p->a); assert(ait != node_lookup.end());
            bit = node_lookup.find(p->b); assert(bit != node_lookup.end());
            
            /*debug() << "visit" << *ait->second <<"\n"<< ait->second->p->globalTransform() << "\n"
                    << "to" << *bit->second <<"\n"<< bit->second->p->globalTransform();
            {
                PrintNode printer;
                visitBetweenNodes(ait->second, bit->second, printer);
            }*/
            
            // Get the error
            IncrementalPose actual_end_relative_to_start;
            uint32_t num_nodes = visitBetweenNodes(
                ait->second, bit->second, AccumulatePose(actual_end_relative_to_start)
            );
            
            IncrementalPose error = p->a_to_b - actual_end_relative_to_start;
            // !!! TODO: better angle normalisation
            while(error.x[2] > M_PI)
                error.x[2] -= M_PI*2;
            while(error.x[2] < -M_PI)
                error.x[2] += M_PI*2;

            // if the error is too big (>45 degrees, or >3m and >4* the
            // distance) ignore it:
            float error_sqlength = error.x[0]*error.x[0] + error.x[1]*error.x[1]; 
            float current_sqlength = actual_end_relative_to_start.x[0]*actual_end_relative_to_start.x[0] +
                                     actual_end_relative_to_start.x[1]*actual_end_relative_to_start.x[1];
            
            sum_weighted_error += error_sqlength * p->weight;
            p->weight = 5.0 / (5 + error_sqlength);

            debug(3) << "error over loop:" << error << "(=" << p->a_to_b << "-"
                     << actual_end_relative_to_start << ")";

            if(error_sqlength > 9 && error_sqlength > current_sqlength*9){
                debug(3) << "ignore constraint: distance error too big";
                ignored_constraints++;
                continue;
            }
            if(std::fabs(error.x[2])*(180/M_PI) > 45){
                debug(3) << "ignore constraint: angle error too big";
                ignored_constraints++;
                continue;
            }
            
            assert(num_nodes > 1);
            // distribute the error along the path in the simplest possible
            // way:
            // (Olsen et al & Grisetti et al use weighting schemes based on the
            //  information matrix of each component constraint, but this will
            //  do to start with)
            IncrementalPose d_pose = error * (lambda / num_nodes); // not num_nodes-1 since include the new constraint in distribution of error
            visitBetweenNodes(ait->second, bit->second, AddPose(d_pose));
            
            // sanity checking:
            #if !defined(CAUV_NO_DEBUG)
            IncrementalPose sanity_check_actual_end_relative_to_start;
            visitBetweenNodes(
                ait->second, bit->second, AccumulatePose(sanity_check_actual_end_relative_to_start)
            );
            IncrementalPose sanity_check_error = p->a_to_b - actual_end_relative_to_start;            
            float sanity_check_error_sqlen = sanity_check_error.x[0]*sanity_check_error.x[0] +
                                             sanity_check_error.x[1]*sanity_check_error.x[1];
            assert(sanity_check_error_sqlen <= error_sqlength + 1e-8); // allow for a tiiiny bit of numerical error
            #endif // !defined(CAUV_NO_DEBUG)
        }
        
        // error is from previous iteration...
        info() << "iter:" << iter-1 << "weighted error:" << sum_weighted_error
               << "ignored:" << ignored_constraints << "/" << constraints.size()
               << "=" << 100*ignored_constraints/constraints.size() << "% constraints";

        lambda *= 0.5;
    }

    // now convert back to global pose
    visitDepthFirst(root, ApplyPose());
}


