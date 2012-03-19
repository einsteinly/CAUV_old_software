// SerialProtocol.cpp : Defines the entry point for the console application.
//
#include "cube3D.h"
#include "SDL/SDL.h"
#include "SDL/SDL_opengl.h"

//----------------------------------------------------------------------//
//- OpenGL cube external global variables                              -//
//----------------------------------------------------------------------//

// Our rotation quaternion (w,x,y,z)
float cube3DRotQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

//----------------------------------------------------------------------//
//- OpenGL cube internal global variables                              -//
//----------------------------------------------------------------------//

// List of our textures path name
const char texturesPath[6][32] = {	"data/front.bmp",
									"data/back.bmp",
									"data/left.bmp",
									"data/right.bmp",
									"data/top.bmp",
									"data/bottom.bmp" };

// List of our OpenGL textures
uint32 textures[6] = {0,0,0,0,0,0};

// Light 0 parameters
float ambientLight[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
float diffuseLight[]	= { 1.0f, 1.0f, 1.0f, 1.0f };
float specularLight[]	= { 1.0f, 1.0f, 1.0f, 1.0f };
float position[]		= {-1.0f,-1.0f, 0.0f, 1.0f };

//----------------------------------------------------------------------//
//- OpenGL cube internal operations                                    -//
//----------------------------------------------------------------------//

// Load our OpenGL textures
bool cube3DLoadTextures(void)
{
	SDL_Surface *sdlImages[6];
	bool sucess = TRUE;
	uint32 i;

	//
	// Load each SDL image
	//
	for (i=0; i<6; i++)
	{
		sdlImages[i] = SDL_LoadBMP(texturesPath[i]);

		if (!sdlImages[i])
		{
			fprintf(stderr, "Unable to load file: %s\n", texturesPath[i]);
		}
	}

	//
	// Load OpenGL texture and free SDL images when we have used it
	//
	for (i=0; i<6; i++)
	{
		if (sdlImages[i])
		{
			//
			// Ask OpenGL to generate a texture Id and bind it
			//
			glGenTextures(1, (GLuint*)&textures[i]);
			glBindTexture(GL_TEXTURE_2D, textures[i]);

			//
			// send the texture to OpenGL and define bi-linear texture filtering
			//
			glTexImage2D(	GL_TEXTURE_2D, 0, 3, sdlImages[i]->w, sdlImages[i]->h, 0,
							GL_BGR, GL_UNSIGNED_BYTE, sdlImages[i]->pixels );
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			//
			// Free our SDL surface
			//
			SDL_FreeSurface(sdlImages[i]);
		}
		else
		{
			textures[i] = 0;
			sucess = FALSE;
		}
	}
	return sucess;
}

// Draw a 3D textured cube
void cube3DDrawCube(void)
{
	//
	// Front
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_FRONT]);
	glNormal3f(-1.0f,0,0);
	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -1.0f, -1.0f, -1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -1.0f, -1.0f,  1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -1.0f,  1.0f,  1.0f );
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -1.0f,  1.0f, -1.0f );
	glEnd();
	
	//
	// Back
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_BACK]);
	glNormal3f(1.0f,0,0);
	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f(  1.0f, -1.0f, -1.0f );
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f(  1.0f,  1.0f, -1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f(  1.0f,  1.0f,  1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f(  1.0f, -1.0f,  1.0f );
	glEnd();
	
	//
	// Left
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_LEFT]);
	glNormal3f(0,-1.0f,0);
	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -1.0f, -1.0f, -1.0f );
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f(  1.0f, -1.0f, -1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f(  1.0f, -1.0f,  1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f( -1.0f, -1.0f,  1.0f );
	glEnd();
	
	//
	// Right
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_RIGHT]);
	glNormal3f(0,1.0f,0);
	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -1.0f,  1.0f, -1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f( -1.0f,  1.0f,  1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f(  1.0f,  1.0f,  1.0f );
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f(  1.0f,  1.0f, -1.0f );
	glEnd();
	
	//
	// Top
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_TOP]);
	glNormal3f(0,0,1.0f);
	glBegin(GL_QUADS);
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -1.0f, -1.0f,  1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f(  1.0f, -1.0f,  1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f(  1.0f,  1.0f,  1.0f );
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -1.0f,  1.0f,  1.0f );
	glEnd();
	
	//
	// Bottom
	//
	glBindTexture(GL_TEXTURE_2D,textures[TEXTURE_BOTTOM]);
	glNormal3f(0,0,-1.0f);
	glBegin(GL_QUADS);
		glTexCoord2f( 1.0f, 0.0f ); glVertex3f( -1.0f, -1.0f, -1.0f );
		glTexCoord2f( 0.0f, 0.0f ); glVertex3f( -1.0f,  1.0f, -1.0f );
		glTexCoord2f( 0.0f, 1.0f ); glVertex3f(  1.0f,  1.0f, -1.0f );
		glTexCoord2f( 1.0f, 1.0f ); glVertex3f(  1.0f, -1.0f, -1.0f );
	glEnd();
}

//----------------------------------------------------------------------//
//- OpenGL cube operations                                             -//
//----------------------------------------------------------------------//

// Initialise our 3D cube
bool cube3DInit(void)
{
	//
	// Define common OpenGL stats
	//
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f );
	glClearDepth(1.0f);
	glDisable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glColor4f(1,1,1,1);

	//
	// Enable lighting and light 0
	//
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//
	// Define our lighting settings for light 0
	//
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);

	//
	// Define and enable color material
	//
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	//
	// Load our OpenGL textures
	//
	return cube3DLoadTextures();
}

// Destroy our OpenGL 3D cube system
void cube3DDestroy(void)
{
	uint32 i;

	//
	// Release OpenGL textures
	//
	for (i=0; i<6; i++)
	{
		if (textures[i])
		{
			//
			// Delete one texture
			//
			glDeleteTextures(1, (GLuint*)&textures[i]);
			textures[i] = 0;
		}
	}
}

// Reshape OpenGL viewport for the specified window size
void cube3DReshape(uint32 width, uint32 height)
{
	//
	// Setup OpenGL viewport
	//
	glViewport(0, 0, width, height);

	//
	// Set up projection matrix
	//
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (float)width/(float)height, 0.1f, 100.0f);

	//
	// Inverse our Texture matrix to avoid Microsoft BMP flip
	//
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	glScalef(1,-1.0f,1);

	//
	// Switch to modelview matrix and reset it
	//
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Draw our OpenGL scene
void cube3DDrawScene(void)
{
	float rotMatrix[16];

	//
	// Clear our color and depth buffer
	//
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//
	// Reset our modelView
	//
	glLoadIdentity();

	//
	// Define our light position (position is transformed by ModelView matrix)
	//
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	//
	// Define our camera point of view
	//
	glTranslatef(0.0f, 0.0f, -5.0f);
	
	//
	// OpenGL is using a Y up vector and the IG-500 is using the mathematical Z up vector representation
	//
	glRotatef( 90,0,1,0);
	glRotatef(-90,1,0,0);
	glRotatef(180,0,0,1);
	
	//
	// Define the modelView matrix according to our rotation quaternion
	//
	glPushMatrix();
	cube3DCreateMatrixFromQuat(cube3DRotQuat, rotMatrix);
	glMultMatrixf(rotMatrix);

	//
	// Draw a 3D cube representing the IG-500 attitude
	//
	cube3DDrawCube();

	//
	// Restore our modelView matrix
	//
	glPopMatrix();

	//
	// Swap our back buffer to front
	//
	SDL_GL_SwapBuffers();
}

// Create a rotation matrix from a rotation quaternion
void cube3DCreateMatrixFromQuat(const float *q, float* pMatrix)
{
	// First row
	pMatrix[ 0] = 1.0f - 2.0f * ( q[2] * q[2] + q[3] * q[3] );
	pMatrix[ 1] = 2.0f * (q[1] * q[2] + q[3] * q[0]);
	pMatrix[ 2] = 2.0f * (q[1] * q[3] - q[2] * q[0]);
	pMatrix[ 3] = 0.0f;
	
	// Second row
	pMatrix[ 4] = 2.0f * ( q[1] * q[2] - q[3] * q[0] );
	pMatrix[ 5] = 1.0f - 2.0f * ( q[1] * q[1] + q[3] * q[3] );
	pMatrix[ 6] = 2.0f * (q[3] * q[2] + q[1] * q[0] );
	pMatrix[ 7] = 0.0f;

	// Third row
	pMatrix[ 8] = 2.0f * ( q[1] * q[3] + q[2] * q[0] );
	pMatrix[ 9] = 2.0f * ( q[2] * q[3] - q[1] * q[0] );
	pMatrix[10] = 1.0f - 2.0f * ( q[1] * q[1] + q[2] * q[2] );
	pMatrix[11] = 0.0f;

	// Fourth row
	pMatrix[12] = 0;
	pMatrix[13] = 0;
	pMatrix[14] = 0;
	pMatrix[15] = 1.0f;
}
