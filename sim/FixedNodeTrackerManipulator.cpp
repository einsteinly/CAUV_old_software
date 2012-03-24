/* 
 * adapted from NodeTrackerManipulator as below
 *
 * -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield
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

#include <sim/FixedNodeTrackerManipulator.h>
#include <osg/Quat>
#include <osg/Notify>
#include <osg/Transform>
#include <iterator>

using namespace cauv;
using namespace osg;
using namespace osgGA;


FixedNodeTrackerManipulator::FixedNodeTrackerManipulator()
    : inherited()
{
}


FixedNodeTrackerManipulator::FixedNodeTrackerManipulator( const FixedNodeTrackerManipulator& m, const CopyOp& copyOp )
    : inherited( m, copyOp ),
      _trackNodePath( m._trackNodePath )
{
}


void FixedNodeTrackerManipulator::setTrackNodePath(const osg::NodePath& nodePath)
{
    _trackNodePath.setNodePath(nodePath);
}

void FixedNodeTrackerManipulator::setTrackNode(osg::Node* node)
{
    if (!node)
    {
        OSG_NOTICE<<"FixedNodeTrackerManipulator::setTrackNode(Node*):  Unable to set tracked node due to null Node*"<<std::endl;
        return;
    }

    osg::NodePathList nodePaths = node->getParentalNodePaths();
    if (!nodePaths.empty())
    {
        if (nodePaths.size()>1)
        {
            OSG_NOTICE<<"osgGA::NodeTrackerManipualtor::setTrackNode(..) taking first parent path, ignoring others."<<std::endl;
        }

        for(unsigned int i=0; i<nodePaths.size(); ++i)
        {
            OSG_NOTICE<<"NodePath "<<i<<std::endl;
            for(NodePath::iterator itr = nodePaths[i].begin();
                itr != nodePaths[i].end();
                ++itr)
            {
                OSG_NOTICE<<"     "<<(*itr)->className()<<std::endl;
            }
        }
        

        OSG_INFO<<"FixedNodeTrackerManipulator::setTrackNode(Node*"<<node<<" "<<node->getName()<<"): Path set"<<std::endl;
        setTrackNodePath( nodePaths[0] );
    }
    else
    {
        OSG_NOTICE<<"FixedNodeTrackerManipulator::setTrackNode(Node*): Unable to set tracked node due to empty parental path."<<std::endl;
    }


}


void FixedNodeTrackerManipulator::computeHomePosition()
{
    osg::Node* node = getTrackNode();
    if(node)
    {
        const osg::BoundingSphere& boundingSphere=node->getBound();

        setHomePosition(boundingSphere._center+osg::Vec3d( 0.0,-3.5f * boundingSphere._radius,0.0f),
                        boundingSphere._center,
                        osg::Vec3d(0.0f,0.0f,1.0f),
                        _autoComputeHomePosition);
    }
}


void FixedNodeTrackerManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    //osg::Vec3d eye,center,up;
    //matrix.getLookAt(eye,center,up,_distance);
    //computePosition(eye,center,up);
}

void FixedNodeTrackerManipulator::setByInverseMatrix(const osg::Matrixd& matrix)
{

}

void FixedNodeTrackerManipulator::computeNodeWorldToLocal(osg::Matrixd& worldToLocal) const
{
    osg::NodePath nodePath;
    if (_trackNodePath.getNodePath(nodePath))
    {
        worldToLocal = osg::computeWorldToLocal(nodePath);
    }
}

void FixedNodeTrackerManipulator::computeNodeLocalToWorld(osg::Matrixd& localToWorld) const
{
    osg::NodePath nodePath;
    if (_trackNodePath.getNodePath(nodePath))
    {
        localToWorld = osg::computeLocalToWorld(nodePath);
    }

}

void FixedNodeTrackerManipulator::computeNodeCenterAndRotation(osg::Vec3d& nodeCenter) const
{
    osg::Matrixd localToWorld, worldToLocal;
    computeNodeLocalToWorld(localToWorld);
    computeNodeWorldToLocal(worldToLocal);

    osg::NodePath nodePath;
    if (_trackNodePath.getNodePath(nodePath) && !nodePath.empty())
        nodeCenter = osg::Vec3d(nodePath.back()->getBound().center())*localToWorld;
    else
        nodeCenter = osg::Vec3d(0.0f,0.0f,0.0f)*localToWorld;
}

void FixedNodeTrackerManipulator::setRotation(osg::Vec3d axis1, float angle1, osg::Vec3d axis2, float angle2, osg::Vec3d axis3, float angle3)
{
    //make quaternion for each euler rotation and multiply them together
    localCameraRotation.makeRotate(angle1, axis1, angle2, axis2, angle3, axis3);
}


void FixedNodeTrackerManipulator::setTranslation(float dist_x,float dist_y,float dist_z)
{
    osg::Vec3d localTranslate(dist_x, dist_y, dist_z);
    
    localCameraTranslation = osg::Matrixd::translate(localTranslate);
}

osg::Matrixd FixedNodeTrackerManipulator::getMatrix() const
{
    osg::Vec3d nodeCenter;
    osg::Quat nodeRotation;
    computeNodeCenterAndRotation(nodeCenter);
    osg::Matrixd rotationMatrix;
    localCameraRotation.get(rotationMatrix);

    return rotationMatrix*osg::Matrixd::translate(nodeCenter)*localCameraTranslation;
}   //translatelocal*translatenodecenter*rotation

osg::Matrixd FixedNodeTrackerManipulator::getInverseMatrix() const
{
    /*osg::Vec3d nodeCenter;
    osg::Quat nodeRotation;
    computeNodeCenterAndRotation(nodeCenter) ;
    return osg::Matrixd::translate(-nodeCenter)*osg::Matrixd::rotate(nodeRotation.inverse())*osg::Matrixd::rotate(_rotation.inverse())*osg::Matrixd::translate(0.0,0.0,-_distance);
    */
    return osg::Matrixd::inverse(getMatrix()); //wont work for all rotations since not using quaternions
}

void FixedNodeTrackerManipulator::computePosition(const osg::Vec3d& eye,const osg::Vec3d& center,const osg::Vec3d& up)
{
}
