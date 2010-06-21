#ifndef __SONAR_OBSERVER_H__
#define __SONAR_OBSERVER_H__

#include <vector>
#include <SDL/SDL.h>


class SonarObserver
{
public:
	virtual void onReceiveImage(uint32_t radius, const CauvImage& img) = 0;
};

class DisplaySonarObserver : public SonarObserver
{
private:
	SDL_Surface *m_screen;
	void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);
	void drawImage(const CauvImage& img);
public:
	DisplaySonarObserver();
	virtual void onReceiveImage(uint32_t radius, const CauvImage& img);

	friend void *displayThread(void *data);
};

class SpreadSonarObserver : public SonarObserver
{
	protected:
		vector<SpreadSocket*> m_sockets;
		uint32_t m_camera_id;
	public:
		SpreadSonarObserver(uint32_t camera_id);
		virtual void onReceiveImage(uint32_t radius, const CauvImage& img);
		void addSocket(TCPSocket *s);
		void removeSocket(TCPSocket *s);
};

#endif //__SONAR_OBSERVER_H__
