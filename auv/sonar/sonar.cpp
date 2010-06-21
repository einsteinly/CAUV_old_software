#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <vector>
#include <math.h>
#include <signal.h>
#include <time.h>

#include <common/cauv_global.h>

#include "sonar_seanet_packet.h"
#include "sonar.h"

using namespace std;

static const unsigned int sonar_image_size = 400;

/* Fill with some sensible defaults */
SonarHeadParams::SonarHeadParams()
{
	v3bParams = 0x01;
	hdCtrl = HDCTRL_ADC8ON | HDCTRL_REPLYASL | HDCTRL_HASMOT
		   | HDCTRL_RAW | HDCTRL_SCANRIGHT | HDCTRL_CONT;
	hdType = 11;
	txnChn1 = 0;
	txnChn2 = 0;
	rxnChn1 = 0;
	rxnChn2 = 0;
	txPulseLen = 0;
	rangeScale = 60, // 6 metres
	leftLim = 2400,  // scan a 90 degree sector...
	rightLim = 4000; // ...straight ahead
	adSpan = 38;
	adLow = 40;
	igainChn1 = 84;
	igainChn2 = 84;
	slopeChn1 = 0;
	slopeChn2 = 0;
	moTime = 25;
	stepSize = 16;
	adInterval = 20;
	nBins = 500;  // Range of about 2.7m
	maxAdBuf = 500;
	lockout = 100;
	minAxisDir = 1600, // 1/16 Grad
	majAxisPan = 1;
	ctl2 = 0;
	scanZ = 0;
	//.v3bData = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
	/* These next values I stole by sniffing the windows example program
	v3b_adSpnChn1 = 0x45;
	v3b_adSpnChn2 = 0x32;
	v3b_adLowChn1 = 0x44;
	v3b_adLowChn2 = 0;
	v3b_igainChn1 = 0x5A;
	v3b_igainChn2 = 0x64;
	v3b_adcSetPChn1 = 0;
	v3b_adcSetPChn2 = 0;
	v3b_slopeChn1 = 0x6400;
	v3b_slopeChn2 = 0;
	v3b_slopeDelayChn1 = 0;
	v3b_slopeDelayChn2 = 0;*/
}

SonarData::SonarData(SonarHeadParams &params)
{
	int binsPerByte = 2;
	int nr_angles; // Number of diff angles the sonar could be staring at

	if (params.hdCtrl & HDCTRL_ADC8ON) {
		binsPerByte = 1;
	} else {
		cauv_global::error("4-bit mode not supported by sonar yet");
	}

	nr_angles = ceil(6400.0f / params.stepSize);
	m_num_angles = nr_angles;
	m_data = new unsigned char*[nr_angles];

	m_num_bins = params.nBins / binsPerByte;
	for(int i = 0; i < nr_angles; i++) {
		m_data[i] = new unsigned char[params.nBins / binsPerByte];
	}

	cout << "Allocated " << (nr_angles * params.nBins / binsPerByte) / 1024 << "kB of memory for sonar" << endl;
}

SonarData::~SonarData()
{
	for (unsigned int i = 0; i < m_num_angles; i++) {
		delete[] m_data[i];
	}

	delete[] m_data;
}

SonarControlMessageObserver::SonarControlMessageObserver(Sonar *sonar)
{
	m_sonar = sonar;
}

void SonarControlMessageObserver::onMessage(const AuvMessage& auvmsg)
{
	switch(auvmsg.getType()) {
        case CMSG_SONAR_CONTROL:
		{
            SonarControlMessage m = dynamic_cast<const SonarControlMessage&>(auvmsg);
            m_sonar->set_scan_settings(m.getMode(), m.getDirection(), m.getWidth(), m.getGain(), m.getRange(), m.getRadialRes(), m.getAngularRes(), m.getUpdateRate());
        }
			break;
        default:
			break;
	}
}

SonarTelemetryMessageObserver::SonarTelemetryMessageObserver(Sonar *sonar)
{
	m_sonar = sonar;
}

void SonarTelemetryMessageObserver::onMessage(const AuvMessage& auvmsg)
{
	switch(auvmsg.getType()) {
		case CMSG_TELEMETRY:
		{
			TelemetryMessage m = dynamic_cast<const TelemetryMessage&>(auvmsg);
			vector<TelemetryEntry> entries = m.getEntries();
			vector<TelemetryEntry>::iterator it = entries.begin();
			while (it < entries.end())
			{
				//cout << (*it) << endl;
				//cout << "Telemetry entry received" << endl;
				if ((*it).getDataContents() == CAUV_TELEMETRY_ATTITUDE) {
					//cout << "Atittude message received!" << endl;
					switch ((*it).getType())
					{
						case CAUV_TYPE_FLOATYPR:
							m_sonar->m_last_attitude = (*it).getData<floatYPR>();
							break;
						default:
							break;
					}
				}
				it++;
			}
		}
			break;
		default:
			break;
	}
}

void Sonar::request_version()
{
	SonarSendversionPacket pkt;
	cout << "Requesting version data..." << endl;
	m_serial_port->sendPacket(pkt);
};

bool MotorState::operator==(MotorState& rhs) const
{

	if (centered != rhs.centered ||
		motoron != rhs.motoron ||
		motoring != rhs.motoring ||
		noparams != rhs.noparams ||
		sentcfg != rhs.sentcfg ||
		cpuid != rhs.cpuid ||
		proglength != rhs.proglength ||
		checksum != rhs.checksum)
		return false;
	return true;
}

MotorState::MotorState() : centered(0), motoron(0), motoring(0), noparams(0),
							sentcfg(0), time(0), cpuid(0), proglength(0),
							checksum(0)
{}

Sonar::Sonar(const char *str ) : m_head_data(0), m_poll_state(POLL_NOTSTARTED), m_img_update_rate(1600), m_cur_data_reqs(0), m_last_attitude(0, 0, 0)
{
	//CauvImage img("common/heatmap.jpg");

	m_img = new CauvImage(sonar_image_size,sonar_image_size);
	m_serial_port = new SerialPort(str);
	m_state = SENDREBOOT;

	/*if (!Sonar::m_heat_map) {
		Sonar::m_heat_map = new u_char[768];
		memcpy(Sonar::m_heat_map, img.getData(), img.getLength());
	}*/
}

u_char *Sonar::m_heat_map = NULL;

Sonar::~Sonar()
{
	delete m_head_data;
}

inline int Sonar::hasParams() const
{
	//FIXME: cheat. Isn't validating parameters properly.
	return !m_motor_state.noparams;
	//return !m_motor_state.noparams && m_motor_state.sentcfg;
}

inline int Sonar::isReady() const
{
	return (m_motor_state.centered &&
			m_motor_state.motoron &&
			!m_motor_state.motoring);
}

/* Send params to the sonar */
void Sonar::upload_params(SonarHeadParams &params)
{
	SonarHeadParamsPacket pkt(params);

	m_next_params = params;
	m_serial_port->sendPacket(pkt);
}

static uint32_t range_from_nbins_adinterval(uint16_t nBins, uint16_t adInterval)
{
	uint32_t half_ping_time = nBins * adInterval * 320; // in nanoseconds
	uint32_t range_mm = (half_ping_time * 15) / 10000; // velocity of sound in mm per second

	return range_mm;
}

/* res has units of cm, range has units of meters */
void Sonar::set_res_range(SonarHeadParams &params, unsigned int res, unsigned int range)
{
	uint16_t nBins;
	uint16_t adInterval;
	float ping_time;

	nBins = (range) / res;

	if (nBins > 1400) {
		cauv_global::error("Someone tried to set the sonar resolution too high.  Reducing it.");
		nBins = 1400;
	}

	ping_time = (2 * range * 1000) / 1500; // units of microseconds. 1500 is VoS
	adInterval = ping_time / (nBins * 0.64); // Sonar has clock time of 640ns

	cout << "Setting nBins to " << nBins << " and adInterval to " << adInterval << endl;
	cout << "Range has been set to " << range_from_nbins_adinterval(nBins, adInterval) << "mm" << endl;

	params.adInterval = adInterval;
	params.nBins = nBins;
	params.rangeScale = range;// * 10;
}

void Sonar::set_scan_settings(u_char mode, uint16_t direction, uint16_t width,
							  u_char gain, uint32_t range, uint32_t radial_res,
							  u_char angular_res, uint16_t update_rate)
{
	SonarHeadParams params;

	cout << "Setting scan settings" << endl;

	/* Continuous scanning */
	if (width == 6400) {
		params.hdCtrl |= HDCTRL_CONT;
		params.hdCtrl &= ~HDCTRL_STARELLIM;
	} else if (width == 0) { // Pinging
		params.hdCtrl &= ~HDCTRL_CONT;
		params.hdCtrl |= HDCTRL_STARELLIM;
	} else { // Sector
		params.hdCtrl &= ~HDCTRL_CONT;
		params.hdCtrl &= ~HDCTRL_STARELLIM;
	}

	params.stepSize = angular_res;
	params.igainChn1 = gain;
	params.igainChn2 = gain;
	params.rangeScale = range;
	m_img_update_rate = update_rate;

	if (direction - width/2u < 0 ) {
		direction += 6400;
	}

	params.leftLim = direction - width/2;
	params.rightLim = params.leftLim + width;

	set_res_range(params, radial_res, range);

	// Updating default parameters?
	switch(mode)
    {
        case CAUV_SONAR_MODE_DEFAULT:
		    m_default_params = params;
            break;
        case CAUV_SONAR_MODE_POLL_IMG:
	    	m_params = params;
    		upload_params(params);
			m_poll_state = POLL_GETBEARING;
            break;
    }
}

/* Takes a pointer to the parent class as input */
void *readThread(void *data)
{
	SonarSeanetPacket *pkt;
	MotorState last_state;
	SonarSenddataPacket sendData;
	Sonar *sonar = (Sonar*)data;

	cout << "Waiting for first sonar packet" << endl;

					cout << "Rebooting sonar" << endl;
					SonarRebootPacket reboot;
					sonar->m_serial_port->sendPacket(reboot);
	while(1)
	{
		again:
		try {
			pkt = sonar->m_serial_port->readPacket();
		} catch (SonarIsDeadException &e) {
			cauv_global::error("Sonar hasn't sent an alive message in 3 secs.  Missing, presumed dead.");
			sonar->m_state = Sonar::SENDREBOOT;
			sonar->m_cur_data_reqs = 0;
			goto again;
		}

		/*cout << "Received packet: ";
		for (int j=0; j < pkt->m_length + 6; j++) {
			printf("%02x ", pkt->m_data[j]);
		}
		cout << endl;
		*//* We want to update our interpretation of the state of the sonar
		 * regardless of what state our state machine is currently in */
		switch  (pkt->m_type) {
			case SEANET_ALIVE:
				cout << "Alive message received" << endl;
				sonar->m_motor_state.centered = pkt->m_data[20] & HDINF_CENTRED;
				sonar->m_motor_state.motoron = pkt->m_data[20] & HDINF_MOTORON;
				sonar->m_motor_state.motoring = pkt->m_data[20] & HDINF_MOTORING;
				sonar->m_motor_state.noparams = pkt->m_data[20] & HDINF_NOPARAMS;
				sonar->m_motor_state.sentcfg = pkt->m_data[20] & HDINF_SENTCFG;
				sonar->m_motor_state.time = *(uint32_t*)&pkt->m_data[14];

				if (!(sonar->m_motor_state == last_state)) {
					printf("Transducer centred: %s\n", B2STR(pkt->m_data[20] & HDINF_CENTRED));
					printf("Motor on: %s\n", B2STR(pkt->m_data[20] & HDINF_MOTORON));
					printf("Motoring: %s\n", B2STR(pkt->m_data[20] & HDINF_MOTORING));
					printf("No params: %s\n", B2STR(pkt->m_data[20] & HDINF_NOPARAMS));
					printf("Sent cfg: %s\n", B2STR(pkt->m_data[20] & HDINF_SENTCFG));
				}

				break;
			case SEANET_VERSIONDATA:
				cout << "Version data received" << endl;
				sonar->m_motor_state.cpuid = *(uint32_t*)&pkt->m_data[14];
				sonar->m_motor_state.proglength = *(uint32_t*)&pkt->m_data[17];
				sonar->m_motor_state.checksum = *(uint16_t*)&pkt->m_data[21];
				break;
			default:
				break;
		}

		/* Update the state machine and implement the protocol */
		switch (sonar->m_state) {
			case Sonar::SENDREBOOT:
				{
					cout << "Rebooting sonar" << endl;
					SonarRebootPacket reboot;
					sonar->m_serial_port->sendPacket(reboot);
					sonar->m_state = Sonar::WAITFORREADY;
					sonar->m_motor_state.centered = 0;
				}
				break;
			case Sonar::WAITFORREADY:
				if (sonar->isReady()) {
					cout << "Moving to state WAITFORVERSION" << endl;
					sonar->m_state = Sonar::WAITFORVERSION;
				}
				break;
			case Sonar::WAITFORVERSION:
				if (pkt->m_type == SEANET_VERSIONDATA) {
					cout << "Moving to state WAITFORPARAMS" << endl;
					sonar->m_state = Sonar::WAITFORPARAMS;
					sonar->upload_params(sonar->m_default_params);
					sonar->m_head_data = new SonarData(sonar->m_default_params);
				} else {
                    static int i = 0;
                    if (++i % 10 == 0) {
    					sonar->request_version();
    					cout << "Requesting version..." << endl;
                    }
				}
				break;
			case Sonar::WAITFORPARAMS:
				if (sonar->hasParams()) {
					sonar->m_state = Sonar::SCANNING;
					cout << "Moving to state SCANNING" << endl;
					//sonar->set_res_range(2, 700);
				} else {
                    cout << "Hasn't confirmed parameters..." << endl;
                }

				break;
			case Sonar::SCANNING:
				/* FIXME:  Maybe do this asynchronously, to ensure we can
				 * always pull data off the sonar fast enough */
				 if (pkt->m_type == SEANET_HEADDATA) {
					sonar->m_cur_data_reqs--;
					// Remember to free the pkt after it's been taken off the queue
					pair<SonarSeanetPacket*, floatYPR> *tmp_pair = new pair<SonarSeanetPacket*, floatYPR> (pkt, sonar->m_last_attitude);
					if (sonar->m_queue.size() > 200) {
						cauv_global::error("Sonar buffer size exceeded");
					} else {
						sonar->m_queue.enqueue(tmp_pair);
					}
				}

				//cout << "Requesting data..." << endl;
				while (sonar->m_cur_data_reqs <= 2) {
					sonar->m_serial_port->sendPacket(sendData);
					sonar->m_cur_data_reqs += 2;
				}
				break;
			default:
				break;
		}

		last_state = sonar->m_motor_state;
	}
}

void *processThread(void *data)
{
	Sonar *sonar = (Sonar*)data;

	while(true)
	{
		// Process!
		pair<SonarSeanetPacket*, floatYPR> *tmp_pair;
		sonar->m_queue.waitAndDequeue(tmp_pair);
		sonar->process_data(tmp_pair->first, tmp_pair->second);

		delete tmp_pair->first;
		delete tmp_pair;
	}
}

void Sonar::addObserver(SonarObserver* o)
{
	m_obs.push_back(o);
}

void Sonar::removeObserver(SonarObserver* o)
{
	vector<SonarObserver*>::iterator i;
	for (i = m_obs.begin(); i != m_obs.end(); i++) {
		if (o == *i) {
			m_obs.erase(i, i+1);
			break;
		}
	}
}

static void set_pixel(CauvImage &img, int x, int y, u_char val)
{
	//u_char r, g, b;

	//r = Sonar::m_heat_map[val * 3];
	//g = Sonar::m_heat_map[val * 3 + 1];
	//b = Sonar::m_heat_map[val * 3 + 2];

    //img.setPixel(x, y, r, g, b);
	img.setPixel(x, y, val, val, val);
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
static void scan_thick_arc(CauvImage &img, int cx, int cy, int radius, float from, float to, u_char value, int thickness)
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

	for(int y=bb.min_y; y <= bb.max_y; y++)
		for(int x=bb.min_x; x <= bb.max_x; x++) {
            int r2 = (x-cx)*(x-cx)+ (y-cy)*(y-cy);
            if (r2 < radius*radius || r2 > (radius+thickness)*(radius+thickness))
                continue;
            
            // TODO: Make this work for to-from >= pi 
            if (ccw((float)cx,(float)cy,from_x,from_y,(float)x,(float)y) * ccw((float)cx,(float)cy,to_x,to_y,(float)x,(float)y) == 1) // Same side
                continue;

			// FIXME:  Hack to make sonar appear oriented correctly
            set_pixel(img, -x, y, value);
        }
}
static void drawLine(CauvImage &img, int x0, int y0, int x1, int y1)
{
	int Dx = x1 - x0; 
	int Dy = y1 - y0;
	int steep = (abs(Dy) >= abs(Dx));
	if (steep) {
		int tmp = x0;
		x0 = y0;
		y0 = tmp;

		tmp = x1;
		x1 = y1;
		y1 = tmp;
		// recompute Dx, Dy after swap
		Dx = x1 - x0;
		Dy = y1 - y0;
	}
	int xstep = 1;
	if (Dx < 0) {
		xstep = -1;
		Dx = -Dx;
	}
	int ystep = 1;
	if (Dy < 0) {
		ystep = -1;
		Dy = -Dy;
	}
	int TwoDy = 2*Dy; 
	int TwoDyTwoDx = TwoDy - 2*Dx; // 2*Dy - 2*Dx
	int E = TwoDy - Dx; //2*Dy - Dx
	int y = y0;
	int xDraw, yDraw;	
	for (int x = x0; x != x1; x += xstep) {		
		if (steep) {			
			xDraw = y;
			yDraw = x;
		} else {			
			xDraw = x;
			yDraw = y;
		}
		// plot
		unsigned char *data = img.getData();
		data[(yDraw * img.getWidth() + xDraw) * 3] = 255;
		data[(yDraw * img.getWidth() + xDraw) * 3+1] = 255;
		data[(yDraw * img.getWidth() + xDraw) * 3+2] = 255;

		// next
		if (E > 0) {
			E += TwoDyTwoDx; //E += 2*Dy - 2*Dx;
			y = y + ystep;
		} else {
			E += TwoDy; //E += 2*Dy;
		}
	}
}static void create_line(CauvImage &img, float grads)
{
	float rads =  (M_PI_2 / 100) * grads;

	int x = (sonar_image_size/2) * cos(rads) + (sonar_image_size/2);
	int y = (sonar_image_size/2) * sin(rads) + (sonar_image_size/2);

	if (x < 0) x = 0;
	if (x > sonar_image_size) x = sonar_image_size;
	if (y < 0) y = 0;
	if (y > sonar_image_size) y = sonar_image_size;

	//cout << "x = " << x << " y = " << y << endl;

	// Draw the scanning line thingy
	drawLine(img, (sonar_image_size/2), (sonar_image_size/2), x, y);
}

bool SonarHeadData::same_settings(SonarHeadData &first, SonarHeadData &second)
{
	// Screw you, DeMorgan.
	return !(first.hdCtrl != second.hdCtrl || first.gain != second.gain ||
			first.adSpan != second.adSpan || first.adLow != second.adLow ||
			first.adInterval != second.adInterval || first.leftLim != second.leftLim ||
			first.rightLim != second.rightLim || first.stepSize != second.stepSize);
}

void Sonar::update_image(SonarHeadData &head_data, floatYPR attitude)
{
	static float last_rads_steprads;

	float rads = (M_PI_2 / 1600) * head_data.bearing - M_PI_2;
	float steprads = (M_PI_2 / 1600) * head_data.stepSize;
	// Offset for data from the INS
	rads -= m_last_attitude.yaw * (M_PI / 180);

	/* 8 bit mode - one char per bin */
	if (head_data.hdCtrl & HDCTRL_ADC8ON) {
		// Calculate which element of the array we should be updating 
		for (int r = 0; r < head_data.dBytes; r++) {
			float distance_along = (r * (sonar_image_size/2) / (float)head_data.dBytes);

			unsigned int scan_number = head_data.bearing / head_data.stepSize;
			
			/*scan_thick_arc(*m_img,400,400,distance_along,rads,rads + steprads,
							m_head_data->m_data[scan_number][r],
							ceil(400/(float)head_data.dBytes));*/
			//FIXME:  Assume angle always increases
			scan_thick_arc(*m_img,(sonar_image_size/2),(sonar_image_size/2),distance_along,last_rads_steprads,rads + steprads,
							m_head_data->m_data[scan_number][r],
							ceil((sonar_image_size/2)/(float)head_data.dBytes));
		}

		last_rads_steprads = rads + steprads;
	} else {
		cauv_global::error("Sonar doesn't support 4-bit mode yet");
	}
}

void Sonar::process_data(SonarSeanetPacket *pkt, floatYPR current_attitude)
{
	static SonarHeadData last_head_data;
	SonarHeadData *head_data;

	// Get the received data from the packet
	head_data = reinterpret_cast<struct SonarHeadData*>(&pkt->getData()[11]);

	// Make sure the data we've got fits into our array
	unsigned int scan_number = head_data->bearing / head_data->stepSize;

	if (scan_number < 0 || scan_number > m_head_data->m_num_angles ||
		head_data->dBytes > m_head_data->m_num_bins)
		return;

	// Store the data in our settings array
	memcpy(m_head_data->m_data[scan_number], &pkt->getData()[44], head_data->dBytes);

	if (!SonarHeadData::same_settings(last_head_data, *head_data)) {
		// Settings have changed.  Clear image and start again.
		m_img->clear();

		switch (m_poll_state) {
			case POLL_GETBEARING:
				m_poll_start_bearing = head_data->bearing;
				if (m_poll_start_bearing < m_next_params.leftLim ||
					m_poll_start_bearing > m_next_params.rightLim)
					m_poll_start_bearing = m_next_params.rightLim;

				m_poll_state = POLL_GOTBEARING;
				break;
			case POLL_GOTBEARING:
				if (head_data->bearing == m_poll_start_bearing) {
					// Scan complete - return the image
					m_poll_state = POLL_NOTSTARTED;
				}
				break;
			default:
				break;
		}
	
		delete m_head_data;
		m_head_data = new SonarData(m_next_params);
		m_params = m_next_params;
	}

	// Drawing stuff
	update_image(*head_data, current_attitude);

	if (head_data->bearing % m_img_update_rate <= head_data->stepSize) {
		//CauvImage scanlineimg(*m_img);
	    //create_line(scanlineimg, -(head_data->stepSize + head_data->bearing) / 16.0 + (m_last_attitude.yaw * 6400) / 360);

		// FIXME:  Assume 8-bit mode again
		uint32_t range_mm = range_from_nbins_adinterval(head_data->dBytes, head_data->adInterval);
		cout << "Sending image with range " << range_mm << "mm" << endl;
		vector<SonarObserver*>::iterator i;
		for (i = m_obs.begin(); i != m_obs.end(); i++) {
			(*i)->onReceiveImage(range_mm, *m_img);
		}
	}

	last_head_data = *head_data;
}

void Sonar::init()
{
	pthread_create(&m_read_thread, NULL, readThread, (void*)this);
	pthread_create(&m_process_thread, NULL, processThread, (void*)this);
}

void sonar_node::on_connect()
{
	cauv_node::on_connect();

	m_tcp_obs->addSocket(m_socket);

	//m_socket->addObserver(new PrintingMessageObserver());
	m_socket->addObserver(new SonarControlMessageObserver(this->m_sonar));
	m_socket->addObserver(new SonarTelemetryMessageObserver(this->m_sonar));

	vector<MsgRequestMask> masks;
	masks.push_back(CMSG_SONAR_CONTROL);
	masks.push_back(CMSG_TELEMETRY);
	request_msg_masks(masks);
}

void sonar_node::on_disconnect()
{
	m_socket->deleteObservers();

	m_tcp_obs->removeSocket(m_socket);
	cauv_node::on_disconnect();
}

sonar_node::sonar_node(const string& device) : cauv_node("Sonar")
{
    m_sonar = new Sonar(device.c_str());
	//m_display_obs = new DisplaySonarObserver();
	m_tcp_obs = new TCPSonarObserver(CAUV_CAM_SONAR);
	m_tcp_processed_obs = new TCPSonarObserver(CAUV_CAM_SONAR_PROCESSED);

	//m_sonar->addObserver(m_display_obs);
	m_sonar->addObserver(m_tcp_obs);
}

sonar_node::~sonar_node()
{
	delete m_sonar;
}

void sonar_node::on_run()
{
	cauv_node::on_run();
	m_sonar->init();
}

sonar_node* s_node;

void cleanup()
{
	cout << "Cleaning up..." << endl;
	cauv_node* oldnode = s_node;
	s_node = 0;
	delete oldnode;
	cout << "Clean up done." << endl;
}

void interrupt(int sig)
{
	cout << "Interrupt caught!" << endl;
	cleanup();
	signal(SIGINT, SIG_DFL);
	raise(sig);
}

int main(int argc, char **argv)
{
	signal(SIGINT, interrupt);
    if (argc != 2)
    {
        cout << "Usage: " << argv[0] << " device" << endl;
        return 1;
    }
    string device(argv[1]);
	s_node = new sonar_node(device);
	s_node->run();
	cleanup();
}

