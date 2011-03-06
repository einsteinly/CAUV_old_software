#ifndef WORLDMODEL_H
#define WORLDMODEL_H

#include <osg/Group>

namespace osg {
    class Vec3f;
    class Vec2f;
}

namespace osgOcean {
    class OceanTechnique;
}

class SceneModel;

namespace cauv {

    class Simulator;
    class SceneModel;


    class WorldModel : public osg::Group
    {
    public:
        WorldModel(osg::Vec2f windDirection, float windSpeed, float depth, float reflectionDamping, float scale, bool isChoppy, float choppyFactor, float foamHeight);

        void setWindDirection(osg::Vec2f wx);
        void setWindSpeed(float ws);
        void setChoppy(bool choppy);
        void setChoppyFactor(float cf);
        void setWaveScale(float ref);
        void setSunPosition(osg::Vec3f pos);
        void setSunDiffuse(osg::Vec3f pos);

    protected:
        osg::ref_ptr<SceneModel> m_scene;
    };

} //namespace cauv

#endif // WORLDMODEL_H
