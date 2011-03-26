#ifndef OCEANSCENE_H
#define OCEANSCENE_H

#include <string>

#include <osg/Referenced>
#include <osg/ref_ptr>

#include <osgOcean/OceanScene>

namespace osgOcean {
    class OceanTechnique;
    class FFTOceanSurface;
}

namespace osg {
    class Vec2f;
    class Vec4f;
    class Group;
    class LightSource;
    class TextureCubeMap;
}

namespace cauv {

    namespace sim {

        class SkyDome;

        class OceanSceneModel : public osg::Referenced
        {

        public:
            OceanSceneModel( const osg::Vec2f& windDirection = osg::Vec2f(1.0f,1.0f), float windSpeed = 12.f,
                             float depth = 10000.f, float reflectionDamping = 0.35f, float waveScale = 1e-8,
                             bool  isChoppy = true, float choppyFactor = -2.5f, float crestFoamHeight = 2.2f,
                             const std::string& textures = "sky_dusk");

            osg::ref_ptr<osgOcean::OceanTechnique> getOceanSurface( );
            osg::ref_ptr<osg::Group> getScene();
            osg::ref_ptr<osgOcean::OceanScene> getOceanScene();
            osg::ref_ptr<osg::LightSource> getSun();
            osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler();

            osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& textures );
            osg::Vec4f intColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255 );

        protected:
            osg::ref_ptr<osg::Group> m_scene;
            osg::ref_ptr<osgOcean::OceanScene> m_oceanScene;
            osg::ref_ptr<osgOcean::FFTOceanSurface> m_oceanSurface;
            osg::ref_ptr<osg::TextureCubeMap> m_cubemap;
            osg::ref_ptr<SkyDome> m_skyDome;
            osg::ref_ptr<osg::LightSource> m_lightSource;

        };

    } // namespace sim

}// namespace cauv

#endif // OCEANSCENE_H
