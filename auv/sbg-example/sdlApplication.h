#ifndef __SDL_APPLICATION_H__
#define __SDL_APPLICATION_H__

#include "sbgCom/sbgCom.h"


//----------------------------------------------------------------------//
//- SDL operations                                                     -//
//----------------------------------------------------------------------//

/// Create and init our SDL device
bool sdlApplicationCreate(int32 width, int32 height, int32 bpp);

/// Destroy our SDL device
void sdlApplicationDestroy(void);

/// Main sdl application loop (returns FALSE if the application should be ended)
bool sdlApplicationHandle(void);

#endif	// __SDL_APPLICATION_H__
