#ifndef __SIM_ATTENUATOR_H__
#define __SIM_ATTENUATOR_H__

#include <limits>

#include <osg/Node>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/TexGenNode>
#include <osg/Texture2D>
#include <osg/TextureRectangle>

// Some choices for the kind of textures we can use ...
//#define USE_TEXTURE_RECTANGLE

template<typename T>
inline T
nextPowerOfTwo(T k)
{
    if (k == T(0))
        return 1;
    k--;
    for (int i = 1; i < std::numeric_limits<T>::digits; i <<= 1)
        k = k | k >> i;
    return k + 1;
}

class Attenuator : public osg::Referenced {
	public:
		Attenuator(unsigned width, unsigned height) :
			_factor(1),
			_texUnit(2),
			_texWidth(width),
			_texHeight(height),
			_attenuatorEnabled(true),
			_scene(new osg::Group)
		{
			_root = new osg::Group();
			createAttenuation();
		}

		osg::Node* createQuad()
		{
			osg::Geometry* geometry = new osg::Geometry;
			{
				osg::Vec3Array* vertices = new osg::Vec3Array;
				{
					vertices->push_back(osg::Vec3f(0, 0, 0));
					vertices->push_back(osg::Vec3f(0, 1, 0));
					vertices->push_back(osg::Vec3f(1, 1, 0));
					vertices->push_back(osg::Vec3f(1, 0, 0));
				}

				osg::Vec3Array* colors = new osg::Vec3Array;
				{
					colors->push_back(osg::Vec3(1, 1, 1));
				}
      
				osg::Vec2Array* texcoords = new osg::Vec2Array;
				{
					texcoords->push_back(osg::Vec2f(0, 0));
					texcoords->push_back(osg::Vec2f(0, 1));
					texcoords->push_back(osg::Vec2f(1, 1));
					texcoords->push_back(osg::Vec2f(1, 0));
				}
				
				geometry->setVertexArray(vertices);
				geometry->setColorArray(colors);
				geometry->setTexCoordArray(0, texcoords);
      
				geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
			}
      
			geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
      
			osg::Geode* geode = new osg::Geode;
			{
				geode->addDrawable(geometry);
			}
      
			return geode;
		}

		void createAttenuation()
		{
			_root->removeChildren(0, _root->getNumChildren());

			// If not enabled, just use the top level camera
			if (!_attenuatorEnabled)
			{
				_root->addChild(_scene.get());
				return;
			}

			_compositeCamera = new osg::Camera;
			{
				_compositeCamera->setName("Composite");
				_compositeCamera->setDataVariance(osg::Object::DYNAMIC);
				_compositeCamera->setInheritanceMask(osg::Camera::READ_BUFFER | osg::Camera::DRAW_BUFFER);
				_compositeCamera->setRenderOrder(osg::Camera::POST_RENDER);
				_compositeCamera->setComputeNearFarMode(osg::Camera::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
				_compositeCamera->setClearMask(0);

				_compositeCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
				_compositeCamera->setViewMatrix(osg::Matrix());
				_compositeCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1, 0, 1));

				osg::StateSet* ss = _compositeCamera->getOrCreateStateSet();
				ss->setBinName("TraversalOrderBin");
				ss->setRenderBinMode(osg::StateSet::USE_RENDERBIN_DETAILS);
			}

			_root->addChild(_compositeCamera.get());

#ifdef USE_TEXTURE_RECTANGLE
            _depthTexture = new osg::TextureRectangle;
#else
            _depthTexture = new osg::Texture2D;
#endif
            _depthTexture->setTextureSize(_texWidth, _texHeight);
			_depthTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
			_depthTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
            _depthTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            _depthTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            _depthTexture->setInternalFormat(GL_DEPTH_COMPONENT);
            _depthTexture->setInternalFormat(GL_DEPTH_COMPONENT24);

			// Then, the actual camera
            osg::Camera* camera = new osg::Camera;
            camera->setName("Scene camera");
    
            camera->setDataVariance(osg::Object::DYNAMIC);
            camera->setInheritanceMask(osg::Camera::ALL_VARIABLES);
            camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER);
            camera->setRenderOrder(osg::Camera::PRE_RENDER);
            camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            camera->setClearColor(osg::Vec4f(0, 0, 0, 0));
            camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

#ifdef USE_TEXTURE_RECTANGLE
            _colorTexture = new osg::TextureRectangle;
#else
            _colorTexture = new osg::Texture2D;
#endif
            _colorTexture->setTextureSize(_texWidth, _texHeight);
            _colorTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            _colorTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
            _colorTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER);
            _colorTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER);
            _colorTexture->setInternalFormat(GL_RGBA);
        
            camera->attach(osg::Camera::COLOR_BUFFER, _colorTexture);
            camera->attach(osg::Camera::DEPTH_BUFFER, _depthTexture);
				
            osg::TexGenNode* texGenNode = new osg::TexGenNode;
            {
                texGenNode->setReferenceFrame(osg::TexGenNode::ABSOLUTE_RF);
                texGenNode->setTextureUnit(_texUnit);
                texGenNode->getTexGen()->setMode(osg::TexGen::EYE_LINEAR);
            }
			
            texGenNode->addChild(_scene.get());
            camera->addChild(texGenNode);
            _root->addChild(camera);

            osg::Node* quad = createQuad();
            {
                osg::StateSet* ss = quad->getOrCreateStateSet();
                ss->setTextureAttributeAndModes(0, _colorTexture.get(), osg::StateAttribute::ON);
                ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
                ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
            }
            _compositeCamera->insertChild(0, quad);
		}

		void setScene(osg::Node* scene)
		{
			_scene->removeChildren(0, _scene->getNumChildren());
			_scene->addChild(scene);
		}

		osg::Node* getRoot()
		{
			return _root.get();
		}

		void resize(int width, int height)
		{
#if !defined(USE_TEXTURE_RECTANGLE) && defined(USE_NON_POWER_OF_TWO_TEXTURE)
			width = nextPowerOfTwo(width);
			height = nextPowerOfTwo(height);
#endif
			_depthTexture->setTextureSize(width, height);
			_colorTexture->setTextureSize(width, height);

			_texWidth = width;
			_texHeight = height;

			createAttenuation();
		}

		void setFactor(float factor)
		{
			if (factor == _factor)
				return;
			_factor = factor;
			createAttenuation();
		}
		unsigned getFactor() const
		{
			return _factor;
		}

		void setTexUnit(unsigned texUnit)
		{
			if (texUnit == _texUnit)
				return;
			_texUnit = texUnit;
			createAttenuation();
		}

		void setAttenuatorEnabled(bool attenuatorEnabled)
		{
			if (attenuatorEnabled == _attenuatorEnabled)
				return;
			_attenuatorEnabled = attenuatorEnabled;
			createAttenuation();
		}
		bool getAttenuatorEnabled() const
		{
			return _attenuatorEnabled;
		}

	protected:
	
		float _factor;
		unsigned _texUnit;
		unsigned _texWidth;
		unsigned _texHeight;
		bool _attenuatorEnabled;

		// The root node that is handed over to the viewer
		osg::ref_ptr<osg::Group> _root;

		// The scene that is displayed
		osg::ref_ptr<osg::Group> _scene;

		// The final camera that composites the pre rendered texture to the final picture
		osg::ref_ptr<osg::Camera> _compositeCamera;

	#ifdef USE_TEXTURE_RECTANGLE
		osg::ref_ptr<osg::TextureRectangle> _depthTexture;
		osg::ref_ptr<osg::TextureRectangle> _colorTexture;
	#else
		osg::ref_ptr<osg::Texture2D> _depthTexture;
		osg::ref_ptr<osg::Texture2D> _colorTexture;
	#endif
};

class AttenuatorEventHandler : public osgGA::GUIEventHandler
{
	public:
		AttenuatorEventHandler(Attenuator* attenuator) :
			_attenuator(attenuator)
		{ }

		/** Handle events, return true if handled, false otherwise. */
		virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&, osg::Object*, osg::NodeVisitor*)
		{
			if (ea.getEventType() == osgGA::GUIEventAdapter::RESIZE) {
				_attenuator->resize(ea.getWindowWidth(), ea.getWindowHeight());
				return true;
			}

			if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
				switch (ea.getKey()) {
				case 'a':
					_attenuator->setAttenuatorEnabled(!_attenuator->getAttenuatorEnabled());
					return true;
				case 'm':
					_attenuator->setFactor(_attenuator->getFactor() * 1.1);
					return true;
				case 'n':
					_attenuator->setFactor(_attenuator->getFactor() / 1.1);
					return true;
				default:
					return false;
				};
			}

			return false;
		}

	protected:
		osg::ref_ptr<Attenuator> _attenuator;
};

#endif
