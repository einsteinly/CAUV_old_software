#ifndef __CUBE_3D_H__
#define __CUBE_3D_H__

#include "sbgCom/sbgCom.h"

//----------------------------------------------------------------------//
//- OpenGL cube definitions                                            -//
//----------------------------------------------------------------------//
#define TEXTURE_FRONT	0
#define TEXTURE_BACK	1
#define TEXTURE_LEFT	2
#define TEXTURE_RIGHT	3
#define TEXTURE_TOP		4
#define TEXTURE_BOTTOM	5

//----------------------------------------------------------------------//
//- OpenGL cube internal operations                                    -//
//----------------------------------------------------------------------//

/// Load our OpenGL textures
bool cube3DLoadTextures(void);

/// Draw a 3D textured cube
void cube3DDrawCube(void);

//----------------------------------------------------------------------//
//- OpenGL cube operations                                             -//
//----------------------------------------------------------------------//

/// Initialise our OpenGL 3D cube system
bool cube3DInit(void);

/// Destroy our OpenGL 3D cube system
void cube3DDestroy(void);

/// Reshape OpenGL viewport for the specified window size
void cube3DReshape(uint32 width, uint32 height);

/// Draw our OpenGL scene
void cube3DDrawScene(void);

/// Create a rotation matrix from a rotation quaternion
void cube3DCreateMatrixFromQuat(const float *q, float* pMatrix);

//----------------------------------------------------------------------//
//- OpenGL cube external global variables                              -//
//----------------------------------------------------------------------//

/// Our rotation quaternion (w,x,y,z)
extern float cube3DRotQuat[4];

#endif	// __CUBE_3D_H__
