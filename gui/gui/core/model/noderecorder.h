#ifndef NODERECORDER_H
#define NODERECORDER_H

#include <boost/make_shared.hpp>
#include <boost/signals/trackable.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <gui/core/model/nodes.h>

namespace cauv {
    namespace gui {

        template<class T>

        class DataRecorder {

        public:
            DataRecorder<T>(const unsigned int samples = 100000, const unsigned int frequency = 1):
                    m_numSamples(samples), m_sampleFrequency(frequency) {
            };

            void setNumSamples(const unsigned int samples){
                boost::mutex::scoped_lock lock(m_mutex);
                m_numSamples = samples;
                if(m_numSamples < m_history.size()) {
                    m_timestamps.resize(m_numSamples);
                    m_history.resize(m_numSamples);
                }
            }

            unsigned int getNumSamples(){
                boost::mutex::scoped_lock lock(m_mutex);
                return m_numSamples;
            }

            void setSampleFrequency(const unsigned int frequency){
                {
                    boost::mutex::scoped_lock lock(m_mutex);
                    m_sampleFrequency = frequency;
                    m_frequencyCount = frequency; // set to frequency as ooposed to 0 so we include the next sample
                }
                this->clear();
            }

            unsigned int getSampleFrequency() {
                return m_sampleFrequency;
            }

            const std::vector<T> getHistory() const {
                boost::mutex::scoped_lock lock(m_mutex);
                return m_history;
            }

            const std::vector<boost::posix_time::ptime> getTimestamps() const {
                boost::mutex::scoped_lock lock(m_mutex);
                return m_timestamps;
            }

            void clear() {
                boost::mutex::scoped_lock lock(m_mutex);
                m_history.clear();
                m_timestamps.clear();
            }

            virtual void update(const T &data) {
                update(data, boost::posix_time::microsec_clock::local_time());
            };

            virtual void update(const T &data, const boost::posix_time::ptime timestamp) {
                boost::mutex::scoped_lock lock(m_mutex);

                if (!m_history.empty() && m_history.size() > m_numSamples) {
                    m_history.erase(m_history.begin());
                    m_timestamps.erase(m_timestamps.begin());

                }

                if(++m_frequencyCount >= m_sampleFrequency) {
                    m_history.push_back(data);
                    m_timestamps.push_back(timestamp);
                    m_frequencyCount = 0;
                }
            };

        protected:
            boost::mutex m_mutex;
            std::vector<T> m_history;
            std::vector<boost::posix_time::ptime> m_timestamps;
            unsigned int m_numSamples;
            unsigned int m_sampleFrequency;
            unsigned int m_frequencyCount;
        };

    } // namesapce gui
} // namespace cauv

#endif // NODERECORDER_H
