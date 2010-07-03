/** Daemon process to connect to the audio hardware, listen for pings from the
  * acoustic pinger beacon, and attempt to compute the radial to the pinger.
  *
  * Uses the JACK audio server/API to acquire the hydrophone signals, the 
  * GNU Scientific Library for DSP operations, and Intel Threading Building
  * Blocks for threading/parallelism.
  *
  * Hugo Vincent, February 2010.
  */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <JackMM.h>
#include <jack/ringbuffer.h>
#include <tbb/concurrent_queue.h>

//#define NO_RECORDING_AUDIO_FILES

#ifndef NO_RECORDING_AUDIO_FILES
#include <sndfile.h>
#endif

#include <gsl/gsl_sf_bessel.h>
#include <Eigen/Array>

typedef jack_default_audio_sample_t sample_t; // Note: this is a 32-bit float
typedef unsigned int uint;

const uint SAMPLE_RATE = 96000;
const uint INP_BUF_DEPTH = 0.1 * SAMPLE_RATE; // 0.1 seconds
const uint FILT_BUF_DEPTH = 0.01 * SAMPLE_RATE; // 10 milliseconds
const uint POWER_CHECK_THRESHOLD = 1e-4; // FIXME tune this!!

// IIR filter for detecting pings (see MATLAB code for filter design details)
const sample_t IIR_COEFFS_FF[] = {1.0, -1.411903338875441, 0.996732850597504};
const sample_t IIR_COEFFS_FB[] = {0.001633574701248, 0.0, -0.001633574701248};

#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)

class PingerJackClient : public clJackClient
{
	public:
		typedef enum {
			FinalizeWavFiles,
			// FIXME more commands
		} PingerCommand;

	private:
		const uint m_numPorts;
		uint64_t m_bytesProcessed;
		clJackPort	**m_inputPorts;
		jack_ringbuffer_t **m_inputBuffers;
		sample_t *m_filterBuffer, *m_workingBuffer;
#ifndef NO_RECORDING_AUDIO_FILES
		SNDFILE *wav_files[4];
#endif

		tbb::concurrent_queue<std::string> m_msgQueue;
		tbb::concurrent_queue<PingerCommand> m_cmdQueue;

		// FIXME:
		Eigen::Vector2d test;

	public:
		double GetMBytesProcessed() { return m_bytesProcessed / 1024.0 / 1024.0; }
		bool PopMessage(std::string& s) { return m_msgQueue.try_pop(s); }
		void SendCommand(PingerCommand cmd) { m_cmdQueue.push(cmd); } 

		PingerJackClient()
			: m_numPorts(2),
			  m_bytesProcessed(0)
		{
			jack_options_t options = JackNullOption;
			jack_status_t status;
			const char *server_name = NULL;

			Open("PingerListenD", options, &status, server_name);
			if (status & JackServerFailed) {
				fprintf (stderr, "Unable to connect to JACK server\n");
				exit (1);
			}
			if (status & JackNameNotUnique)
				throw clXJack("Unique JACK client name already in use\n");

			// Create input ports (i.e. inputs to this process)
			m_inputPorts = new clJackPort*[m_numPorts];
			char portName[16];
			for (uint j = 0; j < m_numPorts; j++)
			{
				m_inputPorts[j] = new clJackPort();
				snprintf(portName, 16, "hydrophone%d", j);
				m_inputPorts[j]->Register(*this, portName, 
						JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
			}

			// Allocate the input ring buffers
			m_inputBuffers = new jack_ringbuffer_t*[m_numPorts];
			for (uint j = 0; j < m_numPorts; j++)
				m_inputBuffers[j] = jack_ringbuffer_create(INP_BUF_DEPTH
						* sizeof(sample_t));

			// Allocate the buffers in which to copy & filter one incoming channel
			// that is used to search for the ping
			m_filterBuffer = new sample_t[FILT_BUF_DEPTH];
			m_workingBuffer = new sample_t[FILT_BUF_DEPTH];
			for (uint j = 0; j < INP_BUF_DEPTH; j++)
			{
				m_filterBuffer[j] = 0.f;
				m_workingBuffer[j] = 0.f;
			}

#ifndef NO_RECORDING_AUDIO_FILES
			// Record some kind of audio file
			SF_INFO wavFileInfo;
			wavFileInfo.samplerate = SAMPLE_RATE;
			wavFileInfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
			wavFileInfo.channels = 1;
			if (sf_format_check(&wavFileInfo) != SF_TRUE)
			{
				fprintf(stderr, "Can't write to WAV file format.\n");
				exit(1);
			}
	#ifdef __APPLE__
			wav_files[0] = sf_open("/Users/hugo/Desktop/latest_ch1.wav", SFM_WRITE, &wavFileInfo);
			wav_files[1] = sf_open("/Users/hugo/Desktop/latest_ch2.wav", SFM_WRITE, &wavFileInfo);
			wav_files[2] = sf_open("/Users/hugo/Desktop/latest_ch3.wav", SFM_WRITE, &wavFileInfo);
			wav_files[3] = sf_open("/Users/hugo/Desktop/latest_filtered.wav", SFM_WRITE, &wavFileInfo);
	#else
			wav_files[0] = sf_open("/home/cauv/audio/latest_ch1.wav", SFM_WRITE, &wavFileInfo);
			wav_files[1] = sf_open("/home/cauv/audio/latest_ch2.wav", SFM_WRITE, &wavFileInfo);
			wav_files[2] = sf_open("/home/cauv/audio/latest_ch3.wav", SFM_WRITE, &wavFileInfo);
			wav_files[3] = sf_open("/home/cauv/audio/latest_filtered.wav", SFM_WRITE, &wavFileInfo);
	#endif
			if (wav_files[0] == NULL || wav_files[1] == NULL
					|| wav_files[2] == NULL || wav_files[3] == NULL)
			{
				fprintf(stderr, "Can't open hydrophone recording WAV files.\n");
				exit(1);
			}
#endif

			// Start the callbacks from the JACK server
			Activate();

			// Get a list of audio capture hardware available through JACK
			JACKMM_STRINGV hwPorts = clJackPort::GetPorts(*this, NULL, NULL, 
					JackPortIsPhysical | JackPortIsOutput);
			if (hwPorts.size() < m_numPorts)
			{
				fprintf(stderr, "Can't find necessary audio capture hardware\n");
				exit(1);
			}
			for (uint j = 0; j < hwPorts.size(); j++)
				printf("Found hardware input '%s'\n", hwPorts[j].c_str());

			// Connect the JACK ports to hardware ports
			for (uint j = 0; j < m_numPorts; j++)
				clJackPort::Connect(*this, hwPorts[j].c_str(), m_inputPorts[j]->GetName());
			printf("All ports connected.\n");

			// Check sample rate (set by JACK daemon configuration)
			if (this->GetSampleRate() != SAMPLE_RATE)
			{
				fprintf(stderr, "Sample rate is %u Hz, but need %u Hz.\n", 
						this->GetSampleRate(), SAMPLE_RATE);
				exit(1);
			}
		}

		~PingerJackClient()
		{
			CleanUp();
		}

		void CleanUp()
		{
#ifndef NO_RECORDING_AUDIO_FILES
			for (uint j = 0; j < 4; j++)
				if (wav_files[j])
					sf_close(wav_files[j]);
#endif
			printf("Processed %g MB.\nCleaning up...\n", GetMBytesProcessed());
			//Close();
			for (uint j = 0; j < m_numPorts; j++)
			{
				delete m_inputPorts[j];
				jack_ringbuffer_free(m_inputBuffers[j]);
			}
			delete[] m_inputPorts;
			delete[] m_inputBuffers;
			delete[] m_filterBuffer;
		}

		// Note: Process callback will be in a different thread to main()
		bool OnProcess(jack_nframes_t numFrames)
		{
			// First, check command queue
			PingerCommand cmd;
			if (m_cmdQueue.try_pop(cmd))
			{
				switch (cmd)
				{
					case FinalizeWavFiles:
						CleanUp();
						fsync(fileno(stderr));
						exit(1);
						break;
					default:
						break;
				}
			}

			if (numFrames > INP_BUF_DEPTH)
			{
				m_msgQueue.push("Error: Too many frames in audio capture buffer.");
				throw clXJack("Too many frames!");
			}

			// Get audio buffers from JACK and copy into the ring buffers
			uint written = 0;
			for (uint j = 0; j < m_numPorts; j++)
			{
				sample_t *in = (sample_t*)m_inputPorts[j]->GetBuffer(numFrames);

#ifndef NO_RECORDING_AUDIO_FILES
				if (wav_files[j])
					sf_writef_float(wav_files[j], in, numFrames);
#endif

				written = jack_ringbuffer_write(m_inputBuffers[j], (char*)in,
						numFrames * sizeof(sample_t)) / sizeof(sample_t);
				if (written < numFrames)
					m_msgQueue.push("Error: Jack buffer xrun!");
			}

			// Search for ping
			if (ScanForPing(written))
			{
				m_msgQueue.push("Found a ping!!!");
				// IsolatePings();
				// StartProcessingFIXME();
			}

			// Finish up
			m_bytesProcessed += m_numPorts * numFrames * sizeof(sample_t);
			return true;
		}

		// Note: This runs in the JACK callback thread
		bool ScanForPing(uint length)
		{
			jack_ringbuffer_read(m_inputBuffers[0], (char*)m_workingBuffer, length);

			// Alias names for brevity
			sample_t *const y = m_filterBuffer, *const x = m_workingBuffer;

			// Apply IIR filter to most recent numFrames in one channel of
			// the input, storing the filter output in a separate buffer.
			for (uint i = 2; i < length; i++)
			{
				// IIR difference equation: (note IIR_COEFFS_FB[1] == 0.0)
				y[i] = 1.f/IIR_COEFFS_FB[0] * 
					(IIR_COEFFS_FF[0] * x[i] + IIR_COEFFS_FF[1] * x[i-1] 
					 + IIR_COEFFS_FF[2] * x[i-2] - IIR_COEFFS_FB[2] * y[i-2]);

				// Square to get power
				y[i] = y[i] * y[i];

#ifndef NO_RECORDING_AUDIO_FILES
				sf_writef_float(wav_files[3], y, length);
#endif
			}

			// Search through the most recent numFrames of m_inputBuffers for ping
			for (uint i = 0; i < (FILT_BUF_DEPTH - 8); i += 8)
			{
				float powerCheck = 0;
				for (uint k = 0; k < 8; k++)
					powerCheck += y[i + k];
				
				if (powerCheck > POWER_CHECK_THRESHOLD)
					return true;
			}
			return false;
		}

		void OnShutdown()
		{
			printf("Shutting down PingerListenD...\n");
			CleanUp();
			exit(0);
		}
};

// Open a client connection to the JACK server and setup the processing callbacks
PingerJackClient pjc;

void signal_handler(int sig)
{
	fprintf(stderr, "INTERRUPT, attempting to shut down cleanly.\n");
	pjc.SendCommand(PingerJackClient::FinalizeWavFiles);
}

int main(int argc, char *argv[])
{
	// Hook SIGINT so we can shutdown cleanly
	signal(SIGINT, &signal_handler);

	// FIXME it might be better to fork/daemonize here

	// At this point, all the heavy lifting happens in a different thread.
	// So we just sleep this thread and wait until we're killed manually.
	//sleep(-1);

	std::string msg;
	for (;;)
	{
		sleep(1);
		while (pjc.PopMessage(msg))
			printf("Message from worker thread: \"%s\"\n", msg.c_str());
	}

	return 0;
}

