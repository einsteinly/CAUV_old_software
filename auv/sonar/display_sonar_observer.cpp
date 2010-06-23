#include "display_sonar_observer.h"

#include <cmath>
#include <algorithm>

using namespace std;

static void set_pixel(cv::Mat* img, int x, int y, unsigned char val)
{
    if (x < 0 || y < 0 || x >= img->size().width || y >= img->size().height) {
        return;
    }
    img->at<unsigned char>(y, x) = val;
}

struct rect { int min_y, max_y, min_x, max_x; };
static rect get_union(const rect& a, const rect& b)
{
    rect ret;
    ret.min_y = min(a.min_y, b.min_y);
    ret.max_y = max(a.max_y, b.max_y);
    ret.min_x = min(a.min_x, b.min_x);
    ret.max_x = max(a.max_x, b.max_x);
    return ret;
}
static rect get_intersection(const rect& a, const rect& b)
{
    rect ret;
    ret.min_y = max(a.min_y, b.min_y);
    ret.max_y = min(a.max_y, b.max_y);
    ret.min_x = max(a.min_x, b.min_x);
    ret.max_x = min(a.max_x, b.max_x);
    return ret;
}
static rect get_arc_bb(int cx, int cy, int radius, float from, float to)
{
	while (to >= 2*M_PI)
	{
		from -= 2*M_PI;
		to -= 2*M_PI;
	}

    rect ret;
	float from_x = cx + radius*cos(from); //     | <-.
	float from_y = cy + radius*sin(from); //  ___|___ 0
	float to_x = cx + radius*cos(to);     //     |
	float to_y = cy + radius*sin(to);     //     |
	
	if (from < 0 && 0 < to)
		ret.max_x = cx + radius;
	else
		ret.max_x = ceil(max(from_x, to_x));

	if (from < M_PI_2 && M_PI_2 < to)
		ret.max_y = cy + radius;
	else
		ret.max_y = ceil(max(from_y, to_y));
	
	if (from < M_PI && M_PI < to)
		ret.min_x = cx - radius;
	else
		ret.min_x = floor(min(from_x, to_x));

	if (from < M_PI+M_PI_2 && M_PI+M_PI_2 < to)
		ret.min_y = cy - radius;
	else
		ret.min_y = floor(min(from_y, to_y));

    return ret; 
}

template<typename T>
int ccw(T p0_x, T p0_y, T p1_x, T p1_y, T p2_x, T p2_y)
{
	T dx1, dx2, dy1, dy2;
	
	dx1 = p1_x - p0_x; dy1 = p1_y - p0_y;
	dx2 = p2_x - p0_x; dy2 = p2_y - p0_y;

	if (dx1*dy2 > dy1*dx2)
		return +1;
	if (dx1*dy2 < dy1*dx2)
		return -1;
	if ((dx1*dx2 < 0) || (dy1*dy2 < 0))
		return -1;
	if ((dx1*dx1 + dy1*dy1) < (dx2*dx2 + dy2*dy2))
		return +1;
	return 0;
}
static void scan_thick_arc(cv::Mat* img, int cx, int cy, int radius, float from, float to, unsigned char value, int thickness)
{
    if (from > to)
    {
        scan_thick_arc(img, cx, cy, radius, to, from, value, thickness);
        return; 
    }

	//cout << "Start drawing at " << from << endl;
	//cout << "End drawing at " << to << endl;

    rect innerbb = get_arc_bb(cx,cy,radius,from,to);
    rect outerbb = get_arc_bb(cx,cy,radius+thickness,from,to);
    rect bb = get_union(innerbb,outerbb);
    rect safe = get_intersection(innerbb, outerbb);

	float from_x = cx + radius*cos(from); //     | <-.
	float from_y = cy + radius*sin(from); //  ___|___ 0
	float to_x = cx + radius*cos(to);     //     |
	float to_y = cy + radius*sin(to);     //     |

    info() << bb.min_y << bb.max_y << bb.min_x << bb.max_x;
	
    for(int y=bb.min_y; y <= bb.max_y; y++)
		for(int x=bb.min_x; x <= bb.max_x; x++) {
            int r2 = (x-cx)*(x-cx)+ (y-cy)*(y-cy);
            if (r2 < radius*radius || r2 > (radius+thickness)*(radius+thickness))
                continue;
            
            // TODO: Make this work for to-from >= pi 
            if (ccw((float)cx,(float)cy,from_x,from_y,(float)x,(float)y) * ccw((float)cx,(float)cy,to_x,to_y,(float)x,(float)y) == 1) // Same side
                continue;

            set_pixel(img, y, x, value);
        }
}


DisplaySonarObserver::DisplaySonarObserver() : m_img(400,400,CV_8UC1)
{
    cv::namedWindow("Sonar display", CV_WINDOW_AUTOSIZE);
}
DisplaySonarObserver::~DisplaySonarObserver()
{
}
void DisplaySonarObserver::onReceiveDataLine(const SonarDataLine& line)
{
	float rads = (M_PI_2 / 1600) * line.bearing - M_PI_2;
	float steprads = (M_PI_2 / 1600) * line.bearingRange;
   
    float from = rads;
    float to = rads + steprads;

    float cosfrom = cos(from);
    float sinfrom = sin(from);
    float costo = cos(to);    
    float sinto = sin(to);    

    int radius = m_img.size().width/2;
    int bincount = line.data.size();

    float bscale = (float)radius/bincount;
    float cx = radius, cy = radius;

    for (int b = 0; b < bincount; b++) {
        float from_radius = b * bscale;
        float to_radius = (b+1) * bscale;
        
        rect innerbb = get_arc_bb(cx,cy,from_radius,from,to);
        rect outerbb = get_arc_bb(cx,cy,to_radius,from,to);
        rect bb = get_union(innerbb,outerbb);
        //rect safe = get_intersection(innerbb, outerbb);

        for(int y=bb.min_y; y <= bb.max_y; y++)
            for(int x=bb.min_x; x <= bb.max_x; x++) {
                float from_x = cx + from_radius*cosfrom; //     | <-.
                float from_y = cy + from_radius*sinfrom; //  ___|___ 0
                float to_x = cx + from_radius*costo;     //     |
                float to_y = cy + from_radius*sinto;     //     |
                
                int r2 = (x-cx)*(x-cx)+ (y-cy)*(y-cy);
                if (r2 < from_radius*from_radius || r2 > to_radius*to_radius)
                    continue;
                
                // TODO: Make this work for to-from >= pi 
                if (ccw((float)cx,(float)cy,from_x,from_y,(float)x,(float)y) * ccw((float)cx,(float)cy,to_x,to_y,(float)x,(float)y) == 1) // Same side
                    continue;

                set_pixel(&m_img, y, x, line.data[b]);
            }
    }

    
    cv::imshow("Sonar display", m_img);
    cv::waitKey(10);
}

