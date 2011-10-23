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

// $Id: JackMM.cc,v 1.7 2005/03/15 19:58:58 visitor Exp $

/*

   Application interface to JACK
   Copyright (C) 2004-2005 Jussi Laako <jussi@sonarnerd.net>

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA

 */


#include <cstdlib>
#include <cstring>
#include <cerrno>

#include <JackMM.h>


extern "C"
{

	void _wrap_jack_shutdown_cb (void *arg)
	{
		((clJackClient *) arg)->OnShutdown();
	}


	int _wrap_jack_process_cb (jack_nframes_t nframes, void *arg)
	{
		return ((((clJackClient *) arg)->OnProcess(nframes)) ? 0 : -1);
	}


	void _wrap_jack_thread_init_cb (void *arg)
	{
		((clJackClient *) arg)->OnThreadInit();
	}


	void _wrap_jack_freewheel_mode (int freewheel_mode, void *arg)
	{
		((clJackClient *) arg)->OnFreewheelModeChange(
			(freewheel_mode) ? true : false);
	}


	int _wrap_jack_buffer_size (jack_nframes_t nframes, void *arg)
	{
		return ((((clJackClient *) arg)->OnBufferSizeChange(nframes)) ? 0 : -1);
	}


	int _wrap_jack_sample_rate (jack_nframes_t nframes, void *arg)
	{
		return ((((clJackClient *) arg)->OnSampleRateChange(nframes)) ? 0 : -1);
	}


	void _wrap_jack_port_registration (jack_port_id_t port, int i, void *arg)
	{
		((clJackClient *) arg)->OnPortRegistration(port, i);
	}


	int _wrap_jack_graph_order (void *arg)
	{
		return ((((clJackClient *) arg)->OnGraphOrderChange()) ? 0 : -1);
	}


	int _wrap_jack_xrun (void *arg)
	{
		return ((((clJackClient *) arg)->OnXrun()) ? 0 : -1);
	}

}  // extern "C"


// --- clJackClient


void clJackClient::Initialize ()
{
	int iE;

	jack_on_shutdown(jackClient, _wrap_jack_shutdown_cb, this);
	if ((iE = jack_set_process_callback(jackClient,
					_wrap_jack_process_cb, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_process_callback()", iE);
	if ((iE = jack_set_thread_init_callback(jackClient,
					_wrap_jack_thread_init_cb, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_thread_init_callback()", iE);
	if ((iE = jack_set_freewheel_callback(jackClient,
					_wrap_jack_freewheel_mode, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_freewheel_callback()", iE);
	if ((iE = jack_set_buffer_size_callback(jackClient,
					_wrap_jack_buffer_size, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_buffer_size_callback()", iE);
	if ((iE = jack_set_sample_rate_callback(jackClient,
					_wrap_jack_sample_rate, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_sample_rate_callback()", iE);
	if ((iE = jack_set_port_registration_callback(jackClient,
					_wrap_jack_port_registration, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_port_registration_callback()", iE);
	if ((iE = jack_set_graph_order_callback(jackClient,
					_wrap_jack_graph_order, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_graph_order_callback()", iE);
	if ((iE = jack_set_xrun_callback(jackClient,
					_wrap_jack_xrun, this)) != 0)
		throw clXJack("clJackClient::Initialize(): jack_set_xrun_callback()", iE);

	bRun = true;
}


clJackClient::clJackClient ()
{
	jackClient = NULL;
	bRun = false;
}


clJackClient::~clJackClient ()
{
	try
	{
		Close();
	}
	catch (...)
	{ }
}


void clJackClient::Create (const char *cpClientNameP)
{
	char *cpClientName;

	cpClientName = (char *) calloc(jack_client_name_size(), sizeof(char));
	if (cpClientName == NULL)
		throw clXJack("clJackClient::Create(): calloc()");
	strncpy(cpClientName, cpClientNameP, jack_client_name_size() - 1);
	jackClient = jack_client_new(cpClientName);
	free(cpClientName);
	if (jackClient == NULL)
		throw clXJack("clJackClient::Create(): jack_client_new()");

	Initialize();
}


void clJackClient::Open (const char *cpClientNameP,
		jack_options_t jackoptOptions, jack_status_t *jackstatStatus,
		const char *cpServerName, const char *cpServerCmd)
{
	char *cpClientName;

	cpClientName = (char *) calloc(jack_client_name_size(), sizeof(char));
	if (cpClientName == NULL)
		throw clXJack("clJackClient::Open(): calloc()");
	strncpy(cpClientName, cpClientNameP, jack_client_name_size() - 1);
	jackClient = jack_client_open(cpClientName, jackoptOptions,
			jackstatStatus, cpServerName, cpServerCmd);
	free(cpClientName);
	if (jackClient == NULL)
		throw clXJack("clJackClient::Open(): jack_client_open()");

	Initialize();
}


void clJackClient::Close ()
{
	int iE;

	if (jackClient != NULL)
	{
		if ((iE = jack_client_close(jackClient)) != 0)
			throw clXJack("clJackClient::Close(): jack_client_close()", iE);
		jackClient = NULL;
	}
}


void clJackClient::Activate ()
{
	int iE;

	if (jackClient == NULL)
		throw clXJack("clJackClient::Activate(): jackClient == NULL");
	if ((iE = jack_activate(jackClient)) != 0)
		throw clXJack("clJackClient::Activate(): jack_activate()", iE);
}


void clJackClient::Deactivate ()
{
	int iE;

	if (jackClient == NULL)
		throw clXJack("clJackClient::Deactivate(): jackClient == NULL");
	if ((iE = jack_deactivate(jackClient)) != 0)
		throw clXJack("clJackClient::Deactivate(): jack_deactivate()", iE);
}


jack_nframes_t clJackClient::GetSampleRate ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetSampleRate(): jackClient == NULL");
	return jack_get_sample_rate(jackClient);
}


jack_nframes_t clJackClient::GetBufferSize ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetBufferSize(): jackClient == NULL");
	return jack_get_buffer_size(jackClient);
}


const char * clJackClient::GetClientName ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetClientName(): jackClient == NULL");
	return jack_get_client_name(jackClient);
}


void clJackClient::LoadInternal (const char *cpClientNameP,
		const char *cpSOName, const char *cpSOData)
{
	int iJackRes;
	char *cpClientName;

	cpClientName = (char *) calloc(jack_client_name_size(), sizeof(char));
	if (cpClientName == NULL)
		throw clXJack("clJackClient::LoadInternal(): calloc()");
	strncpy(cpClientName, cpClientNameP, jack_client_name_size() - 1);
	iJackRes = jack_internal_client_new(cpClientName, cpSOName, cpSOData);
	free(cpClientName);
	if (iJackRes != 0)
		throw clXJack("clJackClient::LoadInternal(): jack_internal_client_new()", iJackRes);
}


void clJackClient::UnloadInternal (const char *cpClientName)
{
	jack_internal_client_close(cpClientName);
}


bool clJackClient::IsRealtime ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::IsRealtime(): jackClient == NULL");
	if (jack_is_realtime(jackClient))
		return true;
	else
		return false;
}


void clJackClient::SetFreewheel (bool bMode)
{
	int iE;

	if (jackClient == NULL)
		throw clXJack("clJackClient::SetFreewheel(): jackClient == NULL");
	if ((iE = jack_set_freewheel(jackClient, (bMode) ? 1 : 0)) != 0)
		throw clXJack("clJackClient::SetFreewheel(): jack_set_freewheel()", iE);
}


void clJackClient::SetBufferSize (jack_nframes_t nFrames)
{
	int iE;

	if (jackClient == NULL)
		throw clXJack("clJackClient::SetBufferSize(): jackClient == NULL");
	if ((iE = jack_set_buffer_size(jackClient, nFrames)) != 0)
		throw clXJack("clJackClient::SetBufferSize(): jack_set_buffer_size()", iE);
}


float clJackClient::GetXrunDelay ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetXrunDelay(): jackClient == NULL");
	return jack_get_xrun_delayed_usecs(jackClient);
}


float clJackClient::GetMaxDelay ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetMaxDelay(): jackClient == NULL");
	return jack_get_max_delayed_usecs(jackClient);
}


void clJackClient::ResetMaxDelay ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::ResetMaxDelay(): jackClient == NULL");
	jack_reset_max_delayed_usecs(jackClient);
}


jack_nframes_t clJackClient::FramesSinceCycleStart ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::FramesSinceCycleStart(): jackClient == NULL");
	return jack_frames_since_cycle_start(jackClient);
}


jack_nframes_t clJackClient::FrameTime ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::FrameTime(): jackClient == NULL");
	return jack_frame_time(jackClient);
}


jack_nframes_t clJackClient::LastFrameTime ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::LastFrameTime(): jackClient == NULL");
	return jack_last_frame_time(jackClient);
}


float clJackClient::GetCPULoad ()
{
	if (jackClient == NULL)
		throw clXJack("clJackClient::GetCPULoad(): jackClient == NULL");
	return jack_cpu_load(jackClient);
}


// --- clJackPort


clJackPort::clJackPort ()
{
	jackPort = NULL;
}


clJackPort::~clJackPort ()
{
}


void clJackPort::InitializeByName (clJackClient &JackClient,
		const char *cpPortName)
{
	jackClient = JackClient;

	if (jackClient == NULL)
		throw clXJack("clJackPort::InitializeByName(): jackClient == NULL");

	jackPort = jack_port_by_name(jackClient, cpPortName);
	if (jackPort == NULL)
		throw clXJack("clJackPort::InitializeByName(): jack_port_by_name()");
}


void clJackPort::InitializeById (clJackClient &JackClient,
		jack_port_id_t portidPort)
{
	jackClient = JackClient;

	if (jackClient == NULL)
		throw clXJack("clJackPort::InitializeById(): jackClient == NULL");

	jackPort = jack_port_by_id(jackClient, portidPort);
	if (jackPort == NULL)
		throw clXJack("clJackPort::InitializeById(): jack_port_by_id()");
}


void clJackPort::Register (clJackClient &JackClient, const char *cpPortNameP,
		const char *cpPortTypeP, unsigned long ulFlags, unsigned long ulBufferSize)
{
	jackClient = JackClient;

	char *cpPortName;
	char *cpPortType;

	if (jackClient == NULL)
		throw clXJack("clJackPort::Register(): jackClient == NULL");
	cpPortName = (char *) calloc(jack_port_name_size(), sizeof(char));
	cpPortType = (char *) calloc(jack_port_type_size(), sizeof(char));
	if (cpPortName == NULL || cpPortType == NULL)
		throw clXJack("clJackPort::Register(): calloc()");
	strncpy(cpPortName, cpPortNameP, jack_port_name_size() - 1);
	strncpy(cpPortType, cpPortTypeP, jack_port_type_size() - 1);
	jackPort = jack_port_register(jackClient, cpPortName, cpPortType,
			ulFlags, ulBufferSize);
	free(cpPortType);
	free(cpPortName);
	if (jackPort == NULL)
		throw clXJack("clJackPort::Register(): jack_port_register()");
}


void clJackPort::Unregister ()
{
	int iE;

	if (jackPort == NULL)
		throw clXJack("clJackPort::Unregister(): jackPort == NULL");
	if ((iE = jack_port_unregister(jackClient, jackPort)) != 0)
		throw clXJack("clJackPort::Unregister(): jack_port_unregister()", iE);
}


void clJackPort::Disconnect ()
{
	int iE;

	if ((iE = jack_port_disconnect(jackClient, jackPort)) != 0)
		throw clXJack("clJackPort::Disconnect(): jack_port_disconnect()", iE);
}


void * clJackPort::GetBuffer (jack_nframes_t nFrames)
{
	void *vpBufPtr;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetBuffer(): jackPort == NULL");
	vpBufPtr = jack_port_get_buffer(jackPort, nFrames);
	if (vpBufPtr == NULL)
		throw clXJack("clJackPort::GetBuffer(): jack_port_get_buffer()");
	return vpBufPtr;
}


const char * clJackPort::GetName ()
{
	const char *cpPortName;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetName(): jackPort == NULL");
	cpPortName = jack_port_name(jackPort);
	if (cpPortName == NULL)
		throw clXJack("clJackPort::GetName(): jack_port_name()");
	return cpPortName;
}


const char * clJackPort::GetShortName ()
{
	const char *cpPortName;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetShortName(): jackPort == NULL");
	cpPortName = jack_port_short_name(jackPort);
	if (cpPortName == NULL)
		throw clXJack("clJackPort::GetShortName(): jack_port_short_name()");
	return cpPortName;
}


int clJackPort::GetFlags ()
{
	int iPortFlags;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetFlags(): jackPort == NULL");
	iPortFlags = jack_port_flags(jackPort);
	if (iPortFlags == -1)
		throw clXJack("clJackPort::GetFlags(): jack_port_flags()");
	return iPortFlags;
}


const char * clJackPort::GetType ()
{
	const char *cpPortType;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetType(): jackPort == NULL");
	cpPortType = jack_port_type(jackPort);
	if (cpPortType == NULL)
		throw clXJack("clJackPort::GetType(): jack_port_type()");
	return cpPortType;
}


bool clJackPort::IsMine (clJackClient &JackClient)
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::IsMine(): jackPort == NULL");
	return ((jack_port_is_mine(JackClient, jackPort) > 0) ? true : false);
}


int clJackPort::Connections ()
{
	int iConnections;

	if (jackPort == NULL)
		throw clXJack("clJackPort::Connections(): jackPort == NULL");
	iConnections = jack_port_connected(jackPort);
	if (iConnections < 0)
		throw clXJack("clJackPort::Connections(): jack_port_connected()");
	return iConnections;
}


bool clJackPort::IsConnectedTo (const char *cpPortName)
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::IsConnectedTo(): jackPort == NULL");
	return ((jack_port_connected_to(jackPort, cpPortName) > 0) ? true : false);
}


JACKMM_STRINGV clJackPort::GetConnections ()
{
	int iIdx = 0;
	const char **cppConnections;
	JACKMM_STRINGV vstrConnections;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetConnections(): jackPort == NULL");
	cppConnections = jack_port_get_connections(jackPort);
	if (cppConnections != NULL)
	{
		while (cppConnections[iIdx] != NULL)
		{
			vstrConnections.push_back(std::string(cppConnections[iIdx]));
			free((void *) cppConnections[iIdx]);
			iIdx++;
		}
		free(cppConnections);
	}

	return vstrConnections;
}


JACKMM_STRINGV clJackPort::GetAllConnections (clJackClient &JackClient)
{
	int iIdx = 0;
	const char **cppConnections;
	JACKMM_STRINGV vstrConnections;

	if (jackPort == NULL)
		throw clXJack("clJackPort::GetAllConnections(): jackPort == NULL");
	cppConnections = jack_port_get_all_connections(JackClient, jackPort);
	if (cppConnections != NULL)
	{
		while (cppConnections[iIdx] != NULL)
		{
			vstrConnections.push_back(std::string(cppConnections[iIdx]));
			free((void *) cppConnections[iIdx]);
			iIdx++;
		}
		free(cppConnections);
	}

	return vstrConnections;
}

#if 0
void clJackPort::Lock ()
{
	int iE;

	if (jackPort == NULL)
		throw clXJack("clJackPort::Lock(): jackPort == NULL");
	if ((iE = jack_port_lock(jackClient, jackPort)) != 0)
		throw clXJack("clJackPort::Lock(): jack_port_lock()", iE);
}


void clJackPort::Unlock ()
{
	int iE;

	if (jackPort == NULL)
		throw clXJack("clJackPort::Unlock(): jackPort == NULL");
	if ((iE = jack_port_unlock(jackClient, jackPort)) != 0)
		throw clXJack("clJackPort::Unlock(): jack_port_unlock()", iE);
}
#endif

jack_nframes_t clJackPort::GetLatency ()
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::GetLatency(): jackPort == NULL");
	return jack_port_get_latency(jackPort);
}


jack_nframes_t clJackPort::GetTotalLatency ()
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::GetTotalLatency(): jackPort == NULL");
	return jack_port_get_total_latency(jackClient, jackPort);
}


void clJackPort::SetLatency (jack_nframes_t nFrames)
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::SetLatency(): jackPort == NULL");
	jack_port_set_latency(jackPort, nFrames);
}


void clJackPort::SetName (const char *cpPortName)
{
	int iE;

	if (jackPort == NULL)
		throw clXJack("clJackPort::SetName(): jackPort == NULL");
	if ((iE = jack_port_set_name(jackPort, cpPortName)) != 0)
		throw clXJack("clJackPort::SetName(): jack_port_set_name()", iE);
}


void clJackPort::SetMonitor (bool bMonitor)
{
	int iE;

	if (jackPort == NULL)
		throw clXJack("clJackPort::SetMonitor(): jackPort == NULL");
	if ((iE = jack_port_request_monitor(jackPort, (bMonitor) ? 1 : 0)) != 0)
		throw clXJack("clJackPort::SetMonitor(): jack_port_request_monitor()", iE);
}


/*void clJackPort::EnsureMonitor (bool bMonitor)
  {
  int iE;

  if (jackPort == NULL)
  throw clXJack("clJackPort::EnsureMonitor(): jackPort == NULL");
  if ((iE = jack_port_ensure_monitor(jackPort, (bMonitor) ? 1 : 0)) != 0)
  throw clXJack("clJackPort::EnsureMonitor(): jack_port_ensure_monitor()", iE);
  }*/


bool clJackPort::IsMonitoring ()
{
	if (jackPort == NULL)
		throw clXJack("clJackPort::IsMonitoring(): jackPort == NULL");
	return ((jack_port_monitoring_input(jackPort) > 0) ? true : false);
}


// ---


void clJackPort::Connect (clJackClient &JackClient, const char *cpSrc,
		const char *cpDest)
{
	int iE;

	iE = jack_connect(JackClient, cpSrc, cpDest);
	if (iE != 0 && iE != EEXIST)
		throw clXJack("clJackPort::Connect(): jack_connect()", iE);
}


void clJackPort::Disconnect (clJackClient &JackClient, const char *cpSrc,
		const char *cpDest)
{
	int iE;

	if ((iE = jack_disconnect(JackClient, cpSrc, cpDest)) != 0)
		throw clXJack("clJackPort::Disconnect(): jack_disconnect()", iE);
}


JACKMM_STRINGV clJackPort::GetPorts (clJackClient &JackClient,
		const char *cpPortNamePat, const char *cpPortTypePat, unsigned long ulFlags)
{
	int iIdx = 0;
	const char **cppPorts;
	JACKMM_STRINGV strvPorts;

	cppPorts = jack_get_ports(JackClient, cpPortNamePat, cpPortTypePat,
			ulFlags);
	if (cppPorts != NULL)
	{
		while (cppPorts[iIdx] != NULL)
		{
			strvPorts.push_back(std::string(cppPorts[iIdx]));
			// free((void *) cppPorts[iIdx]); // FIXME commented this out HV
			iIdx++;
		}
		free(cppPorts);
	}

	return strvPorts;
}


void clJackPort::SetMonitor (clJackClient &JackClient, const char *cpPortName,
		bool bMonitor)
{
	int iE;

	if ((iE = jack_port_request_monitor_by_name(JackClient,
					cpPortName, (bMonitor) ? 1 : 0)) != 0)
		throw clXJack("clJackPort::SetMonitor(): jack_port_request_monitor_by_name()", iE);
}


void clJackPort::Tie (clJackPort &Src, clJackPort &Dest)
{
	int iE;

	if ((iE = jack_port_tie(Src, Dest)) != 0)
		throw clXJack("clJackPort::Tie(): jack_port_tie()", iE);
}


void clJackPort::Untie (clJackPort &Dest)
{
	int iE;

	if ((iE = jack_port_untie(Dest)) != 0)
		throw clXJack("clJackPort::Untie(): jack_port_untie()", iE);
}
