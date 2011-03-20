#include "oceanscene.h"

#include <osg/ref_ptr>
#include <osg/Vec2f>
#include <osg/Node>
#include <osg/LightSource>

#include <osgUtil/CullVisitor>

#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>

#include "skydome.h"

using namespace cauv;
using namespace cauv::sim;

class CameraTrackCallback: public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            osg::Vec3f centre,up,eye;
            cv->getRenderStage()->getCamera()->getViewMatrixAsLookAt(eye,centre,up);
            osg::MatrixTransform* mt = static_cast<osg::MatrixTransform*>(node);
            mt->setMatrix( osg::Matrix::translate( eye.x(), eye.y(), mt->getMatrix().getTrans().z() ) );
        }

        traverse(node, nv);
    }
};


OceanSceneModel::OceanSceneModel( const osg::Vec2f& windDirection, float windSpeed, float depth, float reflectionDamping, float waveScale,
                        bool  isChoppy, float choppyFactor, float crestFoamHeight, const std::string& textures):
m_scene(new osg::Group)
{
    m_cubemap = loadCubeMapTextures(textures);

    // Set up surface
    m_oceanSurface = new osgOcean::FFTOceanSurface( 64, 256, 17, windDirection, windSpeed, depth, reflectionDamping, waveScale, isChoppy, choppyFactor, 10.f, 256 );

    m_oceanSurface->setEnvironmentMap( m_cubemap.get() );
    m_oceanSurface->setFoamBottomHeight( crestFoamHeight );
    m_oceanSurface->setFoamTopHeight( 3.0f );
    m_oceanSurface->enableCrestFoam( true );
    m_oceanSurface->setLightColor( intColor( 105,138,174 ) );
    // Make the ocean surface track with the main camera position, giving the illusion
    // of an endless ocean surface.
    m_oceanSurface->enableEndlessOcean(true);


    // Set up ocean scene, add surface
    osg::Vec3f sunDir = -osg::Vec3f(520.f, 1900.f, 550.f ) ;
    sunDir.normalize();

    m_oceanScene = new osgOcean::OceanScene( m_oceanSurface.get() );
    m_oceanScene->setLightID(0);
    m_oceanScene->enableReflections(true);
    m_oceanScene->enableRefractions(true);

    // Set the size of _oceanCylinder which follows the camera underwater.
    // This cylinder prevents the clear from being visible past the far plane
    // instead it will be the fog color.
    // The size of the cylinder should be changed according the size of the ocean surface.
    m_oceanScene->setCylinderSize( 1900.f, 4000.f );

    m_oceanScene->setAboveWaterFog(0.0012f, intColor( 244,228,179 ) );
    m_oceanScene->setUnderwaterFog(0.002f,  intColor(44,69,106 ) );
    m_oceanScene->setUnderwaterDiffuse( intColor(44,69,106) );
    m_oceanScene->setUnderwaterAttenuation( osg::Vec3f(0.015f, 0.0075f, 0.005f) );

    m_oceanScene->setSunDirection( sunDir );
    m_oceanScene->enableGodRays(true);
    m_oceanScene->enableSilt(true);
    m_oceanScene->enableUnderwaterDOF(true);
    m_oceanScene->enableDistortion(true);
    m_oceanScene->enableGlare(true);
    m_oceanScene->setGlareAttenuation(0.8f);

    // create sky dome and add to ocean scene
    // set masks so it appears in reflected scene and normal scene
    m_skyDome = new SkyDome( 1900.f, 16, 16, m_cubemap.get() );
    m_skyDome->setNodeMask( m_oceanScene->getReflectedSceneMask() | m_oceanScene->getNormalSceneMask() );

    // add a pat to track the camera
    osg::MatrixTransform* transform = new osg::MatrixTransform;
    transform->setDataVariance( osg::Object::DYNAMIC );
    transform->setMatrix( osg::Matrixf::translate( osg::Vec3f(0.f, 0.f, 0.f) ));
    transform->setCullCallback( new CameraTrackCallback );

    transform->addChild( m_skyDome.get() );

    m_oceanScene->addChild( transform );


    // Create and add fake texture for use with nodes without any texture
    // since the OceanScene default scene shader assumes that texture unit
    // 0 is used as a base texture map.
    osg::Image * image = new osg::Image;
    image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
    *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );

    osg::Texture2D* fakeTex = new osg::Texture2D( image );
    fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
    fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
    fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
    fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);

    osg::StateSet* stateset = m_oceanScene->getOrCreateStateSet();
    stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
    stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
    stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
    stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);




    m_lightSource = new osg::LightSource;
    m_lightSource->setLocalStateSetModes();

    osg::ref_ptr<osg::Light> light = m_lightSource->getLight();
    light->setLightNum(0);
    light->setAmbient( osg::Vec4d(0.3f, 0.3f, 0.3f, 1.0f ));
    light->setDiffuse( intColor( 251, 251, 161 ) );
    light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
    light->setPosition( osg::Vec4f( osg::Vec3f(520.f, 1900.f, 550.f ) , 1.f) ); // point light

    m_scene->addChild( m_lightSource );
    m_scene->addChild( m_oceanScene.get() );
    //m_scene->addChild( sunDebug(_sunPositions[CLOUDY]) );

}

osg::ref_ptr<osgOcean::OceanTechnique> OceanSceneModel::getOceanSurface() {
    return m_oceanSurface;
}

osg::ref_ptr<osg::Group> OceanSceneModel::getScene(){
    return m_scene;
}

osg::ref_ptr<osgOcean::OceanScene> OceanSceneModel::getOceanScene() {
    return m_oceanScene;
}

osg::ref_ptr<osg::LightSource> OceanSceneModel::getSun(){
    return m_lightSource;
}

osg::ref_ptr<osg::TextureCubeMap> OceanSceneModel::loadCubeMapTextures( const std::string& textures )
{
    enum {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

    std::string filenames[6];

    filenames[POS_X] = "resources/textures/"+textures+"/east.png";
    filenames[NEG_X] = "resources/textures/"+textures+"/west.png";
    filenames[POS_Z] = "resources/textures/"+textures+"/north.png";
    filenames[NEG_Z] = "resources/textures/"+textures+"/south.png";
    filenames[POS_Y] = "resources/textures/"+textures+"/down.png";
    filenames[NEG_Y] = "resources/textures/"+textures+"/up.png";

    osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
    cubeMap->setInternalFormat(GL_RGBA);

    cubeMap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
    cubeMap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
    cubeMap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
    cubeMap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);

    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, osgDB::readImageFile( filenames[NEG_X] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, osgDB::readImageFile( filenames[POS_X] ) );
    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, osgDB::readImageFile( filenames[NEG_Y] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, osgDB::readImageFile( filenames[POS_Y] ) );
    cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, osgDB::readImageFile( filenames[NEG_Z] ) );
    cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, osgDB::readImageFile( filenames[POS_Z] ) );

    return cubeMap;
}

osg::Vec4f OceanSceneModel::intColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a )
{
    float div = 1.f/255.f;
    return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
}

osgOcean::OceanScene::EventHandler* OceanSceneModel::getOceanSceneEventHandler()
{
    return m_oceanScene->getEventHandler();
}

