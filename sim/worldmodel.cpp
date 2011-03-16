#include "worldmodel.h"

#include <osg/Vec3f>
#include <osg/Vec2f>
#include <osg/Node>
#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/LightSource>

#include <osgOcean/FFTOceanSurface>

using namespace cauv;


WorldModel::WorldModel(osg::Vec2f windDirection, float windSpeed, float depth, float reflectionDamping,
                       float scale, bool isChoppy, float choppyFactor, float foamHeight):
    m_scene(new OceanSceneModel(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, foamHeight))
{
    //this->addChild(m_scene->getScene());
}

osg::ref_ptr<OceanSceneModel> WorldModel::getOceanSceneModel(){
    return m_scene;
}

void WorldModel::setSunPosition(osg::Vec3f pos){
    m_scene->getSun()->getLight()->setPosition(osg::Vec4f(pos, 1.0f));
}

void WorldModel::setSunDiffuse(osg::Vec3f colour){
    m_scene->getSun()->getLight()->setDiffuse(osg::Vec4f(colour, 1.0f));
}

void WorldModel::setWindDirection(osg::Vec2f wind){
    osgOcean::FFTOceanSurface * surface = static_cast<osgOcean::FFTOceanSurface*> (m_scene->getOceanScene()->getOceanTechnique());
    surface->setWindDirection(wind);
}

void WorldModel::setWindSpeed(float ws){
    osgOcean::FFTOceanSurface * surface = static_cast<osgOcean::FFTOceanSurface*> (m_scene->getOceanScene()->getOceanTechnique());
    surface->setWindSpeed(ws, true);
}

void WorldModel::setChoppy(bool choppy){
    osgOcean::FFTOceanSurface * surface = static_cast<osgOcean::FFTOceanSurface*> (m_scene->getOceanScene()->getOceanTechnique());
    surface->setIsChoppy(choppy, true);
}

void WorldModel::setChoppyFactor(float cf){
    osgOcean::FFTOceanSurface * surface = static_cast<osgOcean::FFTOceanSurface*> (m_scene->getOceanScene()->getOceanTechnique());
    surface->setChoppyFactor(cf, true);
}

void WorldModel::setWaveScale(float scale){
    osgOcean::FFTOceanSurface * surface = static_cast<osgOcean::FFTOceanSurface*> (m_scene->getOceanScene()->getOceanTechnique());
    surface->setWaveScaleFactor(scale, true);
}

void WorldModel::setOceanSurfaceHeight(float height){
    m_scene->getOceanScene()->setOceanSurfaceHeight(height);
}

