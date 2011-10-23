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

#ifndef JACKMM_H
#define JACKMM_H

#include <cstdio>
#include <vector>
#include <string>

#include <jack/jack.h>
#include <jack/statistics.h>

#include <Exception.h>

#define JACKMM_STRINGV          std::vector<std::string>

/**
  Exception class for JACK interface.
 */
class clXJack : public clException
{
	public:
		clXJack (const char *cpErrorMsg) :
			clException(cpErrorMsg)
	{ }
		clXJack (const char *cpErrorMsg, int iErrorCode) :
			clException(cpErrorMsg, iErrorCode)
	{ }
};


/**
  Class for JACK client application.
 */
class clJackClient
{
	void Initialize ();
	protected:
	jack_client_t *jackClient;
	public:
	volatile bool bRun;

	operator jack_client_t * ()
	{ return jackClient; }

	clJackClient ();
	clJackClient (const clJackClient &Src)
	{
		jackClient = Src.jackClient;
	}
	virtual ~clJackClient ();
	/**
	  Attempt to become an external client of the JACK server.

	  \param cpClientNameP Name of JACK client
	  \throw clXJack
	 */
	void Create (const char *);
	/**
	  Open an external client session with a JACK server.

	  \param cpClientNameP Name of JACK client
	  \param jackoptOptions Set of JackOpenOptions
	  \param jackstatStatus Status (if non-NULL)
	  \param cpServerName Server name (if non-NULL)
	  \param cpServerCmd Server command (if non-NULL)
	 */
	void Open (const char *, jack_options_t, jack_status_t * = NULL,
			const char * = NULL, const char * = NULL);
	/**
	  Disconnects an external client from a JACK server.

	  \throw clXJack
	 */
	void Close ();
	/**
	  Tell server that application is ready start processing data.

	  \throw clXJack
	 */
	void Activate ();
	/**
	  Tell server to remove client from the process graph.

	  \throw clXJack
	 */
	void Deactivate ();
	/**
	  Get sample rate.

	  \return Current sample rate
	  \throw clXJack
	 */
	jack_nframes_t GetSampleRate ();
	/**
	  Get buffer size.

	  \return Current buffer size
	  \throw clXJack
	 */
	jack_nframes_t GetBufferSize ();
	/**
	  Get name of the client (in case it is dynamically created).

	  \throw clXJack
	 */
	const char * GetClientName ();
	/**
	  Attempt to load an internal client into the JACK server.

	  \param cpClientNameP Name of the new JACK client
	  \param cpSOName Path and name to the shared object containing JACK client
	  \param cpSOData Data to be passed to the new client
	  \throw clXJack
	 */
	void LoadInternal (const char *, const char *, const char *);
	/**
	  Removed an internal client from a JACK server.

	  \param cpClientName Name of the JACK client to be removed
	 */
	void UnloadInternal (const char *);
	/**
	  Check if JACK is running in realtime mode.

	  \return Is realtime?
	  \throw clXJack
	 */
	bool IsRealtime ();
	/**
	  Set jack freewheeling mode.

	  \param bMode Freewheel?
	  \throw clXJack
	 */
	void SetFreewheel (bool);
	/**
	  Set buffer size.

	  \param nFrames Buffer size
	  \throw clXJack
	 */
	void SetBufferSize (jack_nframes_t);
	/**
	  Get Xrun delay in microseconds.

	  \return Delay in usecs
	  \throw clXJack
	 */
	float GetXrunDelay ();
	/**
	  Get maximum delay since startup or reset.

	  \return Delay in usecs
	  \throw clXJack
	 */
	float GetMaxDelay ();
	/**
	  Reset maximum delay.

	  \throw clXJack
	 */
	void ResetMaxDelay ();
	/**
	  Get the time in frames that has passed since the JACK server
	  began the current process cycle.

	  \return Number of frames since cycle start
	  \throw clXJack
	 */
	jack_nframes_t FramesSinceCycleStart ();
	/**
	  Get an estimate of the current time in frames. This is a
	  running counter, no significance should be attached to its
	  value, but it can be compared to a previously returned value.

	  \return Current time in frames
	  \throw clXJack
	 */
	jack_nframes_t FrameTime ();
	/**
	  Get the frame_time after the last processing of the graph.
	  This is only to be used the process callback.

	  This method can be used to put timestamps generated by
	  FrameTime() in correlation to the current process cycle.

	  \return Time of last processed frame
	  \throw clXJack
	 */
	jack_nframes_t LastFrameTime ();
	/**
	  Get the current CPU load estimated by JACK. This is a running
	  average of the time it takes to execute a full process cycle
	  for all clients as a percentage of the real time available
	  per cycle determined by the current size and sample rate.

	  \return Current CPU load
	  \throw clXJack
	 */
	float GetCPULoad ();
	/*
	   ----------------------------------
	   Callbacks, you can override these.
	   ----------------------------------
	 */
	/**
	  JACK process callback.

	  \note This must exist.

	  \param nFrames Number of frames
	  \return Success
	 */
	virtual bool OnProcess (jack_nframes_t) = 0;
	/**
	  Error message handler.

	  \param cpErrorMsg Error message
	 */
	virtual void OnError (const char *cpErrorMsg)
	{ fprintf(stderr, "jack: %s\n", cpErrorMsg); }
	/**
	  Thread initialization.
	 */
	virtual void OnThreadInit ()
	{ }
	/**
	  Freewheel mode change.

	  \param bMode In freewheel mode?
	 */
	virtual void OnFreewheelModeChange (bool bMode)
	{ }
	/**
	  Buffer size changed.

	  \param nFrames New buffer size
	  \return Success
	 */
	virtual bool OnBufferSizeChange (jack_nframes_t nFrames)
	{ return true; }
	/**
	  Sample rate changed.

	  \param nFrames New sample rate
	  \return Success
	 */
	virtual bool OnSampleRateChange (jack_nframes_t nFrames)
	{ return false; }
	/**
	  Shutdown notification. Will be called asynchronoysly.
	 */
	virtual void OnShutdown ()
	{ bRun = false; }
	/**
	  Xrun notification.

	  \return Success
	 */
	virtual bool OnXrun ()
	{ return true; }
	/**
	  Port (un)registration.

	  \param portidPort Port
	  \param iN ?
	 */
	virtual void OnPortRegistration (jack_port_id_t portidPort, int iN)
	{ }
	/**
	  Graph order has changed.

	  \return Success
	 */
	virtual bool OnGraphOrderChange ()
	{ return true; }
};


/**
  Class for JACK port.
 */
class clJackPort
{
	jack_client_t *jackClient;
	protected:
	jack_port_t *jackPort;
	public:
	operator jack_port_t * ()
	{ return jackPort; }

	clJackPort ();
	clJackPort (const clJackPort &Src)
	{
		jackClient = Src.jackClient;
		jackPort = Src.jackPort;
	}
	virtual ~clJackPort ();
	/**
	  Initialize port object by name.

	  \param JackClient Reference to clJackClient object
	  \param cpPortName Port name
	  \throw clXJack
	 */
	void InitializeByName (clJackClient &, const char *);
	/**
	  Initialize port object by id.

	  \param JackClient Reference to clJackClient object
	  \param portidPort Port ID
	  \throw clXJack
	 */
	void InitializeById (clJackClient &, jack_port_id_t);
	/**
	  Create a new port for the client.

	  \param JackClient Reference to clJackClient object
	  \param cpPortNameP Name of the new port
	  \param cpPortTypeP Type name of the new port
	  \param ulFlags JackPortFlags flags
	  \param ulBufferSize Buffer size (only for non-built-in ports)
	  \throw clXJack
	 */
	void Register (clJackClient &, const char *, const char *,
			unsigned long, unsigned long = 0ul);
	/**
	  Remove the port from the client, disconnecting any existing
	  connections.

	  \throw clXJack
	 */
	void Unregister ();
	/**
	  Disconnect port.

	  \throw clXJack
	 */
	void Disconnect ();
	/**
	  Get pointer to the port buffer.

	  \param nFrames Buffer size
	  \return Pointer to buffer
	  \throw clXJack
	 */
	void * GetBuffer (jack_nframes_t);
	/**
	  Get the full name of the port.

	  \return Full port name
	  \throw clXJack
	 */
	const char * GetName ();
	/**
	  Get the short name of the port.

	  \return Short port name
	  \throw clXJack
	 */
	const char * GetShortName ();
	/**
	  Get flags of the port.

	  \return Flags
	  \throw clXJack
	 */
	int GetFlags ();
	/**
	  Get type string of the port.

	  \return Port type
	  \throw clXJack
	 */
	const char * GetType ();
	/**
	  Check if this port is owned by specified client.

	  \param JackClient Reference to clJackClient object
	  \return Belongs to this client?
	  \throw clXJack
	 */
	bool IsMine (clJackClient &);
	/**
	  Get number of connections to this port.

	  \return Number of connections
	  \throw clXJack
	 */
	int Connections ();
	/**
	  Is this port connected directly to specified port?

	  \param cpPortName Other port's name
	  \return Is connected?
	  \throw clXJack
	 */
	bool IsConnectedTo (const char *);
	/**
	  Get a vector of full port names to which the port is connected.

	  \return Connections
	  \throw clXJack
	 */
	JACKMM_STRINGV GetConnections ();
	/**
	  Get a vector of full port names to which the port is connected. If none, returns NULL.

	  This differs from GetConnections() in two important respects:

	  1) You may not call this method from code that is
	  executed in response to a JACK event. For example,
	  you cannot use it in a clJackClient::OnGraphOrderChange()
	  handler.

	  2) You need not be the owner of the port to get information
	  about its connections.

	  \return Connections
	  \throw clXJack
	 */
	JACKMM_STRINGV GetAllConnections (clJackClient &);
	/**
	  Prevent other objects from changing the connection status of
	  a port. The port must be owned by the calling client.

	  \throw clXJack
	 */
	void Lock ();
	/**
	  Allows other objects to change the connection status of a port.

	  \throw clXJack
	 */
	void Unlock ();
	/**
	  Get the latency (in frames) for a port.

	  \return Latency in frames
	  \throw clXJack
	 */
	jack_nframes_t GetLatency ();
	/**
	  Get the sum of latencies (in frames) of the chain for a port.

	  \return Latency in frames
	  \throw clXJack
	 */
	jack_nframes_t GetTotalLatency ();
	/**
	  Set the latency (in frames) of the port.

	  \param nFrames Latency
	  \throw clXJack
	 */
	void SetLatency (jack_nframes_t);
	/**
	  Modify a port's short name. May be called at any time.

	  \param cpPortName New name for a port
	  \throw clXJack
	 */
	void SetName (const char *);
	/**
	  Enable/disable port monitoring.

	  \param bMonitor Monitoring enabled?
	  \throw clXJack
	 */
	void SetMonitor (bool);
	/**
	  Reference counted monitoring.

	  \param bMonitor Monitoring enabled?
	  \throw clXJack
	 */
	//void EnsureMonitor (bool);
	/**
	  Is monitoring enabled for the port?

	  \return Monitoring enabled?
	  \throw clXJack
	 */
	bool IsMonitoring ();

	/* ----- */

	/**
	  Connect ports.

	  Source must be output port and destination must be input port.

	  \param JackClient Reference to clJackClient object
	  \param cpSrc Source port
	  \param cpDest Destination port
	  \throw clXJack
	 */
	static void Connect (clJackClient &, const char *, const char *);
	/**
	  Remove connection between ports.

	  \param JackClient Reference to clJackClient object
	  \param cpSrc Source port
	  \param cpDest Destination port
	  \throw clXJack
	 */
	static void Disconnect (clJackClient &, const char *, const char *);
	/**
	  Get a vector of ports that match the specified arguments.

	  \param JackClient clJackClient object reference                
	  \param cpPortNamePat Port name pattern (regexp)
	  \param cpPortTypePat Port type pattern (regexp)
	  \param ulFlags Port flags
	  \throw clXJack
	 */
	static JACKMM_STRINGV GetPorts (clJackClient &, const char *,
			const char *, unsigned long);
	/**
	  Enable/disable port monitoring.

	  \param JackClient Reference to clJackClient object
	  \param cpPortName Port name
	  \param bMonitor Monitoring enabled?
	  \throw clXJack
	 */
	static void SetMonitor (clJackClient &, const char *, bool);
	/**
	  Tie two locally owned ports together. (pass-thru)

	  \param Src Source port (input)
	  \param Dest Destination port (output)
	  \throw clXJack
	 */
	static void Tie (clJackPort &, clJackPort &);
	/**
	  Untie two locally owned ports.

	  \param Dest Destination port (output)
	  \throw clXJack
	 */
	static void Untie (clJackPort &);
};

#endif
