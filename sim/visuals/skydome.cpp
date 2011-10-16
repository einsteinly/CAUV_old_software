/* Copyright 2011 Cambridge Hydronautics Ltd.
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

/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include "skydome.h"

#include <osgOcean/ShaderManager>

using namespace cauv;
using namespace cauv::sim;

SkyDome::SkyDome( void )
{
    
}

SkyDome::SkyDome( const SkyDome& copy, const osg::CopyOp& copyop ):
    SphereSegment( copy, copyop )
{

}

SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}

SkyDome::~SkyDome(void)
{
}

void SkyDome::create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap )
{
    compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
    setupStateSet(cubemap);
}

void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
    osg::StateSet* ss = new osg::StateSet;

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
    ss->setAttributeAndModes( createShader().get(), osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform("uEnvironmentMap", 0) );

    setStateSet(ss);
}

osg::ref_ptr<osg::Program> SkyDome::createShader(void)
{
    osg::ref_ptr<osg::Program> program = new osg::Program;

    // Do not use shaders if they were globally disabled.
    if (osgOcean::ShaderManager::instance().areShadersEnabled())
    {
        char vertexSource[]=
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    gl_Position = ftransform();\n"
            "    vTexCoord = gl_Vertex.xyz;\n"
            "}\n";

        char fragmentSource[]=
            "uniform samplerCube uEnvironmentMap;\n"
            "varying vec3 vTexCoord;\n"
            "\n"
            "void main(void)\n"
            "{\n"
            "    vec3 tex = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
            "    gl_FragColor = textureCube( uEnvironmentMap, tex.xzy );\n"
            "  gl_FragColor.a = 0.0;\n"
            "}\n";

        program->setName( "sky_dome_shader" );
        program->addShader(new osg::Shader(osg::Shader::VERTEX,   vertexSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));
    }

    return program;
}
