#include "camera.h"

using namespace cauv;
using namespace cauv::sim;

#include <osg/Texture2D>
#include <osg/Image>
#include <osgText/Text>
#include <osg/Geode>
#include <osg/Geometry>

Camera::Camera(int width, int height) : m_renderTexture(new osg::Texture2D)
{

    //setUpAsHUD();

    /*
    // set the projection matrix
    this->setProjectionMatrix(osg::Matrix::ortho2D(0,width,0,height));

    // set the view matrix
    this->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    this->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    this->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    this->setRenderOrder(osg::Camera::POST_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    this->setAllowEventFocus(false);
    */

    setSize(width, height);
    m_renderTexture->setInternalFormat(GL_RGBA);
    m_renderTexture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    m_renderTexture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);


    getCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    getCamera()->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // We need to render to the texture BEFORE we render to the screen
    getCamera()->setRenderOrder(osg::Camera::PRE_RENDER);

    // The camera will render into the texture that we created earlier
    getCamera()->attach(osg::Camera::COLOR_BUFFER, m_renderTexture);

    getCamera()->setAllowEventFocus(false);
}

void Camera::setSize(int width, int height){
    m_renderTexture->setTextureSize(width, height);
    getCamera()->setViewport(0, 0, width, height);
}

osg::ref_ptr<osg::Texture2D> Camera::getTexture(){
    return m_renderTexture;
}

osg::ref_ptr<osg::Image> Camera::getImage(){
    return m_renderTexture->getImage();
}




void Camera::setUpAsHUD() {
/*
    // set the projection matrix
    this->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));

    // set the view matrix
    this->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    this->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    this->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    this->setRenderOrder(osg::Camera::POST_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    this->setAllowEventFocus(false);




    osg::Geode* geode = new osg::Geode();

    std::string timesFont("fonts/arial.ttf");

    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osg::Vec3 position(150.0f,800.0f,0.0f);
    osg::Vec3 delta(0.0f,-120.0f,0.0f);

    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("Head Up Displays are simple :-)");

        position += delta;
    }


    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("All you need to do is create your text in a subgraph.");

        position += delta;
    }


    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("Then place an osg::Camera above the subgraph\n"
                      "to create an orthographic projection.\n");

        position += delta;
    }

    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("Set the Camera's ReferenceFrame to ABSOLUTE_RF to ensure\n"
                      "it remains independent from any external model view matrices.");

        position += delta;
    }

    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("And set the Camera's clear mask to just clear the depth buffer.");

        position += delta;
    }

    {
        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setPosition(position);
        text->setText("And finally set the Camera's RenderOrder to POST_RENDER\n"
                      "to make sure its drawn last.");

        position += delta;
    }


    {
        osg::BoundingBox bb;
        for(unsigned int i=0;i<geode->getNumDrawables();++i)
        {
            bb.expandBy(geode->getDrawable(i)->getBound());
        }

        osg::Geometry* geom = new osg::Geometry;

        osg::Vec3Array* vertices = new osg::Vec3Array;
        float depth = bb.zMin()-0.1;
        vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
        vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
        vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
        vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
        geom->setVertexArray(vertices);

        osg::Vec3Array* normals = new osg::Vec3Array;
        normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
        geom->setNormalArray(normals);
        geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

        osg::StateSet* stateset = geom->getOrCreateStateSet();
        stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
        //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

        geode->addDrawable(geom);
    }

    this->addChild(geode);

    */
}
