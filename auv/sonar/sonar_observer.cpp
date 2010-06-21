#include <common/cauv_img.h>
#include <SDL/SDL.h>
#include <pthread.h>

#include <common/cauv_message.h>
#include "sonar_observer.h"

#define WIDTH 800
#define HEIGHT 800
#define BPP 4
#define DEPTH 32

void DisplaySonarObserver::drawImage(const CauvImage& img)
{
	unsigned char *data = img.getData();

	for (unsigned int h = 0; h < img.getHeight(); h++) {
		for (unsigned int w = 0; w < img.getWidth(); w++) {
			setPixel(w, h, data[0], data[1], data[2]);
            data += 3;
		}
	}
}

void DisplaySonarObserver::setPixel(int x, int y, unsigned char r,
									unsigned char g, unsigned char b)
{
	uint32_t *pixmem32;
	uint32_t colour;

	colour = SDL_MapRGB(m_screen->format, r, g, b);

	pixmem32 = (uint32_t*) m_screen->pixels + y * m_screen->w + x;
	*pixmem32 = colour;
}

void DisplaySonarObserver::onReceiveImage(uint32_t radius, const CauvImage& img)
{
	if(SDL_MUSTLOCK(m_screen)) {
		if(SDL_LockSurface(m_screen) < 0) return;
	}

	drawImage(img);

	if(SDL_MUSTLOCK(m_screen)) SDL_UnlockSurface(m_screen);

	SDL_Flip(m_screen); 
}

void *displayThread(void *data)
{
	//SDL_Event event;
	DisplaySonarObserver *obs = (DisplaySonarObserver*)data;

	//int keypress = 0;
	
	if (SDL_Init(SDL_INIT_VIDEO) < 0 )
		return (void*)0;
   
	if (!(obs->m_screen = SDL_SetVideoMode(WIDTH, HEIGHT, DEPTH, SDL_HWSURFACE))) {
		SDL_Quit();
		return (void*)0;
	}
/*
	while(!keypress) {
		while(SDL_PollEvent(&event)) {	  
			switch (event.type) {
				case SDL_QUIT:
  					keypress = 1;
  					break;
				case SDL_KEYDOWN:
					keypress = 0;
					break;
			}
		}
	}

	SDL_Quit();
*/	return 0;
}

DisplaySonarObserver::DisplaySonarObserver()
{
	pthread_t thread;
	pthread_create(&thread, NULL, displayThread, (void*)this);
}

TCPSonarObserver::TCPSonarObserver(uint32_t camera_id) : m_camera_id(camera_id)
{
}

void TCPSonarObserver::onReceiveImage(uint32_t radius, const CauvImage& img)
{
	vector<TCPSocket*>::iterator it;
	TCPSocket *socket;

	SonarImageMessage msg(m_camera_id, radius, img);
	ImageMessage img_msg(m_camera_id, img);

	for (it = this->m_sockets.begin(); it != this->m_sockets.end(); it++) {
		socket = *it;
		try {
			if (socket) {
				socket->sendMessage(msg);
				socket->sendMessage(img_msg);
			}
		} catch (ConnectionClosedException &e) {
			cout << "Error sending sonar image: " << e.what() << endl;
		}
	}
}

void TCPSonarObserver::addSocket(TCPSocket *s)
{
	vector<TCPSocket*>::iterator i;
	for (i = m_sockets.begin(); i != m_sockets.end(); i++)
		if (s == *i)
			return;
	m_sockets.push_back(s);
}

void TCPSonarObserver::removeSocket(TCPSocket *s)
{
	vector<TCPSocket*>::iterator i;
	for (i = m_sockets.begin(); i != m_sockets.end(); i++)
		if (s == *i) {
			m_sockets.erase(i, i+1);
			break;
		}
}

