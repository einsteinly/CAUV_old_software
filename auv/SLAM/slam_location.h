#pragma once
#include "graph_optimiser.h"

namespace cauv {

class SlamCloudLocation{
    public:
        typedef std::vector< RelativePose, Eigen::aligned_allocator<RelativePose> > rel_pose_vec;

        SlamCloudLocation()
            : m_relative_to(),
              m_relative_transformation(Eigen::Matrix4f::Identity()) {
            _checkAlignment();
        }

        template<typename PointT>
        explicit SlamCloudLocation(boost::shared_ptr<SlamCloudLocation> const& p)
            : m_relative_to(p->m_relative_to),
              m_relative_transformation(p->m_relative_transformation) {
            _checkAlignment();
        }

        explicit SlamCloudLocation(float x, float y, float theta_radians)
            : m_relative_to(), 
              m_relative_transformation(Eigen::Matrix4f::Identity()) {
            m_relative_transformation.block<2,1>(0,3) = Eigen::Vector2f(x, y);
            m_relative_transformation.block<2,2>(0,0) *= Eigen::Matrix2f(Eigen::Rotation2D<float>(theta_radians));
            _checkAlignment();
        }

        template<typename PointT>
        SlamCloudLocation(boost::shared_ptr<SlamCloudLocation> const& relative_to, Eigen::Matrix4f const& rel_tr)
            : m_relative_to(relative_to), m_relative_transformation(rel_tr) {
            _checkAlignment();
        }

        virtual ~SlamCloudLocation(){
        }

        // post-multiply
        void transform(Eigen::Matrix4f const& transformation){
            m_relative_transformation = m_relative_transformation * transformation;
            transformationChanged();
        }

        void setRelativeToNone(){
            if(m_relative_to){
                m_relative_to.reset();
                transformationChanged();
            }
        }
        void setRelativeTo(location_ptr p){
            if(p != m_relative_to){
                m_relative_to = p;
                transformationChanged();
            }
        }
        
        // add constraint from this -> p (p observed from this) t is the
        // RELATIVE (not incremental) pose at which p is observed
        RelativePose addConstraintTo(location_ptr p, Eigen::Matrix4f const& t){
            const RelativePose r = RelativePose::from4dAffine(t);
            
            m_constrained_to.push_back(p);
            m_constraints.push_back(r);
            
            return r;
        }

        void addConstraintTo(location_ptr p, RelativePose const& r){
            m_constrained_to.push_back(p);
            m_constraints.push_back(r);
        }

        static pose_constraint_ptr addConstraintBetween(location_ptr from,
                                                        location_ptr to,
                                                        Eigen::Matrix4f from_to_to){
            const RelativePose rel_pose = from->addConstraintTo(to, from_to_to);
            return boost::make_shared<RelativePoseConstraint>(rel_pose, from, to);
        }

        void setRelativeTransform(Eigen::Matrix4f const& m){
            m_relative_transformation = m;
            transformationChanged();            
        }

        Eigen::Matrix4f globalTransform() const{
            Eigen::Matrix4f r = relativeTransform();
            location_ptr p = relativeTo();
            if(p)
                return p->globalTransform() * r;
            return r;
        }

        Eigen::Matrix4f const& relativeTransform() const{ return m_relative_transformation; }
        location_ptr relativeTo() const{ return m_relative_to; }

        // !!! constrainedTo should be in constraints
        location_vec const& constrainedTo() const{ return m_constrained_to; }
        rel_pose_vec const& constraints() const{ return m_constraints; }

        // (relatively) expensive: use sparingly
        typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > v3f_vec;
        v3f_vec constraintEndsGlobal() const{
            v3f_vec r;
            for(RelativePose const& rel: m_constraints){
                const Eigen::Matrix4f pt_transform = globalTransform() * rel.to4dAffine();
                const Eigen::Vector3f pt = pt_transform.block<3,1>(0,3);
                r.push_back(pt);
            }
            return r;
        }
        

        // We have an Eigen::Matrix4f as a member
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        virtual void transformationChanged(){}

    private:
        void _checkAlignment() const{
        }
    
        location_ptr    m_relative_to;
        Eigen::Matrix4f m_relative_transformation;

        location_vec  m_constrained_to;
        rel_pose_vec  m_constraints;
};

}
