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
#include <tbb/concurrent_queue.h>

typedef jack_default_audio_sample_t sample_t; // Note: this is a 32-bit float
typedef unsigned int uint;

const uint SAMPLE_RATE = 96000;
const uint INP_BUF_DEPTH = 0.1 * SAMPLE_RATE; // 0.1 seconds

// IIR filter for detecting pings (see MATLAB code for filter design details)
const sample_t IIR_COEFFS_FF[] = {1.0, -1.411903338875441, 0.996732850597504};
const sample_t IIR_COEFFS_FB[] = {0.001633574701248, 0.0, -0.001633574701248};

#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)

class PingerJackClient : public clJackClient
{
	private:
		const uint m_numPorts, m_inputBufferDepth;
		uint64_t m_bytesProcessed;
		clJackPort	**m_inputPorts;
		sample_t **m_inputBuffers, *m_filterBuffer;
		uint *m_inputBufferInIdx, *m_inputBufferOutIdx;
		uint m_filterBufferInIdx, m_filterBufferOutIdx;
		tbb::concurrent_queue<std::string> m_msgQueue;

	public:
		double GetMBytesProcessed() { return m_bytesProcessed / 1024.0 / 1024.0; }
		bool PopMessage(std::string& s) { return m_msgQueue.try_pop(s); }

		PingerJackClient()
			: m_numPorts(2),
			  m_inputBufferDepth(INP_BUF_DEPTH),
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
				snprintf(portName, 16, "input%d", j);
				m_inputPorts[j]->Register(*this, portName, 
						JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
			}

			// Allocate the input ring buffers
			m_inputBuffers      = new sample_t*[m_numPorts];
			m_inputBufferInIdx  = new uint[m_numPorts];
			m_inputBufferOutIdx = new uint[m_numPorts];
			for (uint j = 0; j < m_numPorts; j++)
			{
				m_inputBuffers[j] = new sample_t[m_inputBufferDepth];
				m_inputBufferInIdx[j] = m_inputBufferOutIdx[j] = 0;
			}

			// Allocate the buffer in which to copy & filter one incoming channel
			// that is used to search for the ping
			m_filterBuffer = new sample_t[m_inputBufferDepth];
			m_filterBufferInIdx = m_filterBufferOutIdx = 0;

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
			printf("Processed %g MB.\nCleaning up...\n", GetMBytesProcessed());
			Close();
			for (uint j = 0; j < m_numPorts; j++)
			{
				delete m_inputPorts[j];
				delete[] m_inputBuffers[j];
			}
			delete[] m_inputPorts;
			delete[] m_inputBuffers;
			delete[] m_filterBuffer;
			delete[] m_inputBufferInIdx;
			delete[] m_inputBufferOutIdx;
		}

		// Note: Process callback will be in a different thread to main()
		bool OnProcess(jack_nframes_t numFrames)
		{
			if (numFrames > m_inputBufferDepth)
			{
				m_msgQueue.push("Error: Too many frames in audio capture buffer.");
				throw clXJack("Too many frames!");
			}

			// Get audio buffers from JACK and copy into the ring buffers
			uint count = 0, offset = 0, startIdx = m_inputBufferInIdx[0];
			for (uint j = 0; j < m_numPorts; j++)
			{
				sample_t *in = (sample_t*)m_inputPorts[j]->GetBuffer(numFrames);
				do {
					count = min(m_inputBufferDepth - m_inputBufferInIdx[j], numFrames - offset); 
					memcpy(&in[offset], &(m_inputBuffers[j][m_inputBufferInIdx[j]]),
							count * sizeof(sample_t));
					m_inputBufferInIdx[j] = (m_inputBufferInIdx[j] + count) % m_inputBufferDepth;
					offset += count;
				} while (offset < numFrames);
			}

			// Search for ping
			if (ScanForPing(startIdx, count))
			{
				// IsolatePings();
				// StartProcessingFIXME();
			}

			// Finish up
			m_bytesProcessed += m_numPorts * numFrames * sizeof(sample_t);
			return true;
		}

		// Note: This runs in the JACK callback thread
		bool ScanForPing(uint startIdx, uint length)
		{
			// Alias names for brevity
			sample_t *const x = m_inputBuffers[0], *const y = m_filterBuffer;

			// Apply IIR filter to most recent numFrames in one channel of
			// the input, storing the filter output in a separate buffer.
			for (uint i = 2+startIdx; i < startIdx+length; i++) // FIXME ring buffer indexing foo
			{
				// IIR difference equation: (note IIR_COEFFS_FB[1] == 0.0)
				y[i] = 1.f/IIR_COEFFS_FB[0] * 
					(IIR_COEFFS_FF[0] * x[i] + IIR_COEFFS_FF[1] * x[i-1] 
					 + IIR_COEFFS_FF[2] * x[i-2] - IIR_COEFFS_FB[2] * y[i-2]);

				// Square to get power
				y[i] = y[i] * y[i];
			}

			// Search through the most recent numFrames of m_inputBuffers for ping
			if (0 /* FIXME */)
				return true;

			// msgQueue.push("Nothing to see here");
			return false;
		}

		void OnShutdown()
		{
			printf("Shutting down PingerListenD...\n");
			CleanUp();
			exit(0);
		}
};

int main(int argc, char *argv[])
{
	// Open a client connection to the JACK server and setup the processing callbacks
	PingerJackClient pjc;

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

