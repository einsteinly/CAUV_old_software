#include "sdlApplication.h"
#include "cube3D.h"
#include "SDL/SDL.h"

//----------------------------------------------------------------------//
//- SDL internal global variables                                      -//
//----------------------------------------------------------------------//
SDL_Surface *pSdlDevice = NULL;

//----------------------------------------------------------------------//
//- SDL operations                                                     -//
//----------------------------------------------------------------------//

// Create and init our SDL device
bool sdlApplicationCreate(int32 width, int32 height, int32 bpp)
{
	//
	// First destroy a previous SDL device
	//
	sdlApplicationDestroy();

	//
	// Init SDL system
	//
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		fprintf(stderr, "Video initialization failed: %s\n", SDL_GetError());
		return FALSE;
	}

	//
	// Create our SDL window
	//
	pSdlDevice = SDL_SetVideoMode(width, height, bpp, SDL_OPENGL|SDL_GL_DOUBLEBUFFER|SDL_HWPALETTE|SDL_RESIZABLE|SDL_HWSURFACE);

	//
	// Set OpenGL attributes (enable double buffering)
	//
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	if (pSdlDevice)
	{
		//
		// Initialise our OpenGL 3D cube
		//
		if (cube3DInit())
		{
			//
			// First OpenGL projection and viewport definition
			//
			cube3DReshape(width, height);
			return TRUE;

		}
		else
		{
			fprintf(stderr, "Unable to init our OpenGL cube.\n");
			return FALSE;
		}
	}
	else
	{
		//
		// Unable to create our SDL window
		//
		fprintf(stderr, "Unable to create SDL window: %s\n", SDL_GetError());
		return FALSE;
	}
}

// Destroy our SDL device
void sdlApplicationDestroy(void)
{
	if (pSdlDevice)
	{
		//
		// Destroy our 3D cube
		//
		cube3DDestroy();

		//
		// Destroy SDL device
		//
		SDL_Quit();
		pSdlDevice = NULL;
	}
}

// Main sdl application loop (returns FALSE if the application should be ended)
bool sdlApplicationHandle(void)
{
	SDL_Event sdlEvent;
	bool dontExit = TRUE;

	//
	// Handle SDL events
	//
	while (SDL_PollEvent(&sdlEvent))
	{
		//
		// Handle each event type
		//
		switch (sdlEvent.type)
		{
		case SDL_KEYDOWN:
			if (sdlEvent.key.keysym.sym == SDLK_ESCAPE)
			{
				dontExit = FALSE;
			}
			break;
		case SDL_QUIT:
			dontExit = FALSE;
			break;
		default:
			break;
		}
	}

	//
	// Draw our OpenGL scene
	//
	cube3DDrawScene();
	
	SDL_Delay(10);
	
	return dontExit;
}
