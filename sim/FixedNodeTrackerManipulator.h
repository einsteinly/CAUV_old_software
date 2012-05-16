/* 
 * adapted from NodeTrackerManipulator as below
 *
 * -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#ifndef FIXED_NODE_TRACKER_MANIPULATOR
#define FIXED_NODE_TRACKER_MANIPULATOR 1

#include <osgGA/CameraManipulator>
#include <osg/ObserverNodePath>


namespace cauv {


class FixedNodeTrackerManipulator : public osgGA::CameraManipulator
{
    typedef CameraManipulator inherited;

    public:

        FixedNodeTrackerManipulator();

        FixedNodeTrackerManipulator( const FixedNodeTrackerManipulator& om,
                                const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY );

        void setTrackNodePath(const osg::NodePath& nodePath);
        void setTrackNodePath(const osg::ObserverNodePath& nodePath) { _trackNodePath = nodePath; }
        osg::ObserverNodePath& getTrackNodePath() { return _trackNodePath; }

        void setTrackNode(osg::Node* node);
        osg::Node* getTrackNode() { osg::NodePath nodePath; return _trackNodePath.getNodePath(nodePath) && !nodePath.empty() ? nodePath.back() : 0; }
        const osg::Node* getTrackNode() const { osg::NodePath nodePath; return _trackNodePath.getNodePath(nodePath) && !nodePath.empty()  ? nodePath.back() : 0; }

        virtual void setByMatrix(const osg::Matrixd& matrix);
        virtual void setByInverseMatrix(const osg::Matrixd& matrix);
        virtual osg::Matrixd getMatrix() const;
        virtual osg::Matrixd getInverseMatrix() const;

        void setRotation(osg::Vec3d axis1, float angle1, osg::Vec3d axis2, float angle2, osg::Vec3d axis3, float angle3);
        void setTranslation(osg::Vec3d translation);

        virtual void computeHomePosition();

    protected:
        void computeNodeWorldToLocal(osg::Matrixd& worldToLocal) const;
        void computeNodeLocalToWorld(osg::Matrixd& localToWorld) const;

        void computeNodeCenterAndRotation(osg::Vec3d& center, osg::Quat& rotation) const;

        void computePosition(const osg::Vec3d& eye,const osg::Vec3d& lv,const osg::Vec3d& up);


        osg::ObserverNodePath   _trackNodePath;
        osg::Quat localCameraRotation;
        osg::Matrixd localCameraTranslation;
};

}

#endif 
