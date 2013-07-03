/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "gemini_node.h"

#include <limits>
#include <cmath>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <debug/cauv_debug.h>
#include <common/msg_classes/image.h>
#include <utility/threadsafe-observable.h>
#include <utility/rounding.h>
#include <utility/string.h>
#include <utility/async.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/GeminiStatusMessage.h>
#include <generated/types/SonarImageMessage.h>
#include <generated/types/SpeedOfSoundMessage.h>
#include <generated/types/GeminiControlMessage.h>
#include <generated/message_observers.h>

// Gemini SDK comes last because it pollutes the global namespace
#include "GeminiStructuresPublic.h"
#include "GeminiCommsPublic.h"

namespace cauv{

static float floatTemp(uint16_t temp){
    int16_t signedtemp;
    float  ftemp;
    bool present = temp & 0x8000;
    if(present){
        // 10 bits of data:
        signedtemp = temp & 0x3ff;
        // sign extend
        bool negative = temp & 0x200;
        if(negative)
            signedtemp |= 0xfc00;
        ftemp = signedtemp / 4.0f;
        return ftemp;
    }else{
        return std::numeric_limits<float>::quiet_NaN();
    }
}

static std::string fmtTemp(uint16_t temp){
    if(temp & 0x8000)
        return mkStr() << floatTemp(temp);
    else
        return "X";
}

class GeminiObserver{
    public:
        virtual void onCGemPingHead(CGemPingHead const*, float){}
        virtual void onCGemPingLine(CGemPingLine const*){}
        virtual void onCGemPingTail(CGemPingTail const*, uint32_t){}
        virtual void onCGemPingTailExtended(CGemPingTailExtended const*, uint32_t){}
        virtual void onCGemStatusPacket(CGemStatusPacket const*){}
        virtual void onCGemAcknowledge(CGemAcknowledge const*){}
        virtual void onCGemBearingData(CGemBearingData const*){}
        virtual void onUnknownData(uint8_t const*){}
};

class ReBroadcaster: public GeminiObserver{
    public:
        ReBroadcaster(CauvNode& node)
            : GeminiObserver(),
              m_current_ping_id(0),
              m_current_msg(),
              m_current_raw_data(),
              m_range_lines_start(0),
              m_range_lines_end(0),
              m_range(0),
              m_rangecompression_mult(1),
              m_node(node){
        }

        virtual void onCGemPingHead(CGemPingHead const* h, float range){
            const float sos = h->m_spdSndVel / 10.0f;
            debug(2) << "PingHead:"
                    << "start:" << h->m_startRange
                    << "end:" << h->m_endRange
                    << "numBeams:" << h->m_numBeams
                    << "numChans:" << (int)h->m_numChans
                    << "sampChan:" << (int)h->m_sampChan
                    << "speedOfSound:" << sos;
            // new ping:
            // each line is data from a particular distance away
            uint32_t num_lines = h->m_endRange - h->m_startRange;
            // !!! NB: the DLL doesn't remove range-line compression: so
            // actually the number of lines is fewer than this depending on
            // h->m_rangeCompressionUsed:
            if(h->m_rangeCompressionUsed == 1)
                m_rangecompression_mult = 2;
            else if(h->m_rangeCompressionUsed == 2)
                m_rangecompression_mult = 4;
            else if(h->m_rangeCompressionUsed == 3)
                m_rangecompression_mult = 8;
            else if(h->m_rangeCompressionUsed == 4)
                m_rangecompression_mult = 16;
            else
                m_rangecompression_mult = 1;
            num_lines /= m_rangecompression_mult;
            // each beam is data from a constant bearing
            uint32_t num_beams = h->m_numBeams;
            m_current_ping_id = h->m_pingID;
            m_current_ping_time = now();
            m_range_lines_start = h->m_startRange;
            m_range_lines_end = h->m_endRange;
            m_range = range;
            m_current_msg = boost::make_shared<SonarImageMessage>(
                SonarID::Gemini,
                PolarImage(
                    std::vector<uint8_t>(), // this will be replaced later with downsampled data
                    ImageEncodingType::RAW_uint8_1,
                    computeBearingBins(num_beams),
                    0, // filled in later
                    0, // filled in later
                    0, // filled in later
                    // Check this, (needs converting from whatever the sonar's
                    // time base is into our unix time):
                    TimeStamp(
                        h->m_transmitTimestampL/1e6+h->m_transmitTimestampH*double(100000000)/1e6,
                        h->m_transmitTimestampL % 1000000
                    )
                )
            );
            m_current_raw_data.resize(num_lines*num_beams);
            debug(3) << "New ping: ID=" << m_current_ping_id << "lines:" << num_lines << "beams:" << num_beams;

            // This message also tells us the speed of sound: so broadcast it
            // now:
            m_node.send(boost::make_shared<SpeedOfSoundMessage>(sos));
        }

        virtual void onCGemPingLine(CGemPingLine const* l){
            // accumulate this line of equidistant data:
            int32_t line_id = l->m_lineID;
            int32_t line_idx = line_id - m_range_lines_start;
            uint16_t ping_id = l->m_pingID;
            if((ping_id & 0xff) != (m_current_ping_id & 0xff)){
                debug(2) << "bad pingID";
                return;
            }
            uint32_t num_beams = m_current_msg->image().bearing_bins.size() - 1;
            uint32_t offset = line_idx * num_beams;
            if(offset + num_beams > m_current_raw_data.size()){
                static uint16_t last_error_ping_id = 0;
                if(ping_id != last_error_ping_id){
                    error() << "invalid line index (further errors for this ping will be suppressed)";
                    last_error_ping_id = ping_id;
                }
            }
            for(uint32_t i = 0; i != num_beams; i++)
                m_current_raw_data[offset+i] = ((uint8_t*)&(l->m_startOfData))[i];
        }

        virtual void onCGemPingTailExtended(CGemPingTailExtended const*, uint32_t resize_to_rangelines){
            pingComplete(resize_to_rangelines);
        }

        virtual void onCGemPingTail(CGemPingTail const*, uint32_t resize_to_rangelines){
            pingComplete(resize_to_rangelines);
        }

        virtual void onCGemStatusPacket(CGemStatusPacket const* s){
            boost::shared_ptr<GeminiStatusMessage> r = boost::make_shared<GeminiStatusMessage>();
            r->sonarId(s->m_sonarId);
            r->vccInt(s->m_vccInt);
            r->vccAux(s->m_vccAux);
            r->dcVolt(s->m_dcVolt);
            GeminiTemperatures current(
                ((s->m_dieTemp * 503.975) / 1024.0) - 273.15,
                floatTemp(s->m_vga1aTemp),
                floatTemp(s->m_vga1bTemp),
                floatTemp(s->m_vga2aTemp),
                floatTemp(s->m_vga2bTemp),
                floatTemp(s->m_TX1Temp),
                floatTemp(s->m_TX2Temp),
                floatTemp(s->m_TX3Temp)
            );
            r->currentTemps(current);
            GeminiTemperatures critical(
                ((s->m_dieOverTemp * 503.975) / 1024.0) - 273.15,
                floatTemp(s->m_vga1aShutdownTemp),
                floatTemp(s->m_vga1bShutdownTemp),
                floatTemp(s->m_vga2aShutdownTemp),
                floatTemp(s->m_vga2bShutdownTemp),
                floatTemp(s->m_TX1ShutdownTemp),
                floatTemp(s->m_TX2ShutdownTemp),
                floatTemp(s->m_TX3ShutdownTemp)
            );
            r->criticalTemps(critical);
            switch(s->m_transducerFrequency){
                case 0: r->transducerFrequency(868.0f); break;
                case 1: r->transducerFrequency(723.0f); break;
            }
            r->linkType(s->m_linkType);
            r->BOOTSTSRegister(s->m_BOOTSTSRegister);
            r->shutdownStatus(s->m_shutdownStatus);

            m_node.send(r);
        }

    private:
        static std::vector<int32_t> computeBearingBinsNoCache(uint32_t num_beams){
            std::vector<int32_t> r;
            r.reserve(num_beams+2);
            const double radConvert = (180.0 / M_PI) * (6400.0/360.0) * 0x10000;
            for(uint32_t beam = 0; beam <= num_beams; beam++){
                // see Gemini Interface Specification Appendix B
                double radians = std::asin(((2*(beam+0.5) - num_beams) / num_beams) * 0.86602540);
                r.push_back(round(radConvert * radians));
            }
            return r;
        }
        static std::vector<int32_t> computeBearingBins(uint32_t num_beams){
            static std::vector<int32_t> bearings_256 = computeBearingBinsNoCache(256);
            if(num_beams == 256)
                return bearings_256;
            return computeBearingBinsNoCache(num_beams);
        }

        static uint32_t nextPowerOfTwo(uint32_t n){
            n--;
            n |= n >> 1;
            n |= n >> 2;
            n |= n >> 4;
            n |= n >> 8;
            n |= n >> 16;
            return ++n;
        }

        void pingComplete(uint32_t resize_to_rangelines){
            // Use opencv to downsample the data to the number of rangelines
            // that we were configured to send: The range-compression in the
            // sonar head has already reduced the number of lines by a factor
            // of m_mrangecompression_mult
            uint32_t source_rangelines = (m_range_lines_end - m_range_lines_start) / m_rangecompression_mult;
            uint32_t bearing_beams = m_current_msg->image().bearing_bins.size() - 1;
            cv::Mat full_resolution_image(
                source_rangelines,
                bearing_beams,
                CV_8UC1,
                &m_current_raw_data[0]
            );
            // !!! sneaky: probably okay, and really the only efficient way to
            // do this without introducing multableX members to messages. This
            // is okay because we haven't serialised it yet.
            std::vector<uint8_t> &resized_data = const_cast<std::vector<uint8_t>&>(
                m_current_msg->image().data
            );
            float &rangeStart = const_cast<float&>(m_current_msg->image().rangeStart);
            float &rangeEnd   = const_cast<float&>(m_current_msg->image().rangeEnd);
            float &rangeConv  = const_cast<float&>(m_current_msg->image().rangeConversion);
            // range conversion in full-resolution image
            float full_range_conversion = m_range / m_range_lines_end;
            rangeStart = m_range_lines_start * full_range_conversion;
            rangeEnd   = m_range_lines_end * full_range_conversion;
            // range conversion in downsampled image
            #ifndef NO_EXTRA_GEMSONAR_SUBSAMPLING
            // limit the resizing to a power of two to avoid aliasing:
            resize_to_rangelines = nextPowerOfTwo(resize_to_rangelines);
            rangeConv  = m_range / resize_to_rangelines;
            // This zero-initialises, but that isn't possible to avoid.
            // Hopefully the compiler is smart enough to realise we're setting
            // every element anyway... (it's a long shot):
            resized_data.resize(resize_to_rangelines * bearing_beams);
            cv::Mat downsampled_image(
                resize_to_rangelines,
                bearing_beams,
                CV_8UC1,
                &resized_data[0]
            );
            cv::resize(
                full_resolution_image,
                downsampled_image,
                downsampled_image.size(),
                0, 0, cv::INTER_LINEAR // !!! INTER_AREA might be better
            );
            debug(3) << "sending SonarImageMessage:" << bearing_beams << "x" << resize_to_rangelines;
            #else
            // !!! just rely on range compression, (now I understand how it
            // works...), otherwise all we really do is introduce aliasing:
            rangeConv = m_range / source_rangelines;
            resized_data = m_current_raw_data;
            debug(3) << "sending SonarImageMessage:" << bearing_beams << "x" << source_rangelines;
            #endif
            m_node.send(m_current_msg);
        }

        uint16_t m_current_ping_id;
        TimeStamp m_current_ping_time;
        boost::shared_ptr<SonarImageMessage> m_current_msg;
        std::vector<uint8_t> m_current_raw_data;
        uint32_t m_range_lines_start;
        uint32_t m_range_lines_end;
        float m_range;
        uint32_t m_rangecompression_mult;
        CauvNode& m_node;
};

class GeminiSonar: public ThreadSafeObservable<GeminiObserver>,
                   public MessageObserver,
                   boost::noncopyable{
    private:
        typedef boost::mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;
    public:
        GeminiSonar(uint16_t sonar_id, uint32_t inter_ping_musec=1000000)
            : m_sos(1499.2f),
              m_sonar_id(sonar_id),
              m_inter_ping_musec(inter_ping_musec),
              m_range(0),
              m_gain_percent(0),
              m_range_lines(200),
              m_ping_continuous(false),
              m_conn_state(),
              m_currently_pinging(0),
              m_async_service(boost::make_shared<AsyncService>(1)),
              m_cancel_timeout_mux(),
              m_cancel_timeout(boost::make_shared<bool>(true)),
              m_gem_mux(){
            assert(!the_sonar);
            the_sonar = this;
            m_conn_state.ok = true;
            m_conn_state.initialised = false;
        }

        ~GeminiSonar(){
            lock_t l(m_cancel_timeout_mux);
            if(!*m_cancel_timeout){
                *m_cancel_timeout = true;
                m_gem_mux.unlock();
            }
            m_async_service.reset();
            the_sonar = nullptr;
        }

        float range() const{
            return m_range;
        }

        bool ok() const{
            return m_conn_state.ok;
        }

        bool initialised() const{
            return m_conn_state.initialised;
        }

        void autoConfig(float range, float gain_percent){
            m_range = range;
            m_gain_percent = gain_percent;
            applyConfigAndPing();
        }

        void init(){
            int success = 0;
            m_conn_state.sonarId = 0;
            lock_t l(m_gem_mux);
            success = GEM_StartGeminiNetworkWithResult(m_sonar_id);
            if(!success){
                error() << "Could not start network connection to sonar" << m_sonar_id;
                m_conn_state.ok = false;
            }else{
                // ('Evo' is the native mode for the sonar, John told us to use
                // this mode, SeaNet is there for compatibility with Tritech's
                // old software only. 'EvoC' means use range-line compression
                char mode[] = "EvoC";
                GEM_SetGeminiSoftwareMode(mode);
                GEM_SetHandlerFunction(&CallBackFn);
            }
        }
        
        // postcondition:
        // Either:
        //  An error occurs, and this function returns immediately with no
        //  state changes
        // Or:
        //  m_gem_mux is locked, the config is applied to the sonar, triggering
        //  a ping, and a timeout is setup (after three seconds) in case a ping
        //  is not received from the sonar.
        void applyConfigAndPing(){
            debug(2) << "applyConfigAndPing...";
            if(!initialised()){
                error() << "will not ping: connection not initialised";
                return;
            }else if(!ok()){
                error() << "will not ping: connection not ok";
                return;
            }
            lock_t l(m_cancel_timeout_mux);
            if(!*m_cancel_timeout){
                error() << "will not ping: ping already prepared!";
                return;
            }
            m_gem_mux.lock();
            m_currently_pinging = true;
            GEM_AutoPingConfig(m_range, m_gain_percent, m_sos);
            if(m_range_lines <= 32){
                GEM_SetGeminiEvoQuality(0);
            }else if(m_range_lines <= 64){
                GEM_SetGeminiEvoQuality(1);
            }else if(m_range_lines <= 128){
                GEM_SetGeminiEvoQuality(2);
            }else if(m_range_lines <= 256){
                GEM_SetGeminiEvoQuality(3);
            }else if(m_range_lines <= 512){
                GEM_SetGeminiEvoQuality(4);
            }else if(m_range_lines <= 1024){
                GEM_SetGeminiEvoQuality(5);
            }else if(m_range_lines <= 2048){
                GEM_SetGeminiEvoQuality(6);
            }else if(m_range_lines <= 4096){
                GEM_SetGeminiEvoQuality(7);
            }
            // This seems to cause lock-ups under high load, safer to request
            // another ping when we've received the last one in full
            //GEM_SetPingMode(m_ping_continuous);
            //GEM_SetInterPingPeriod(m_inter_ping_musec);
            GEM_SetPingMode(0);
            debug(3) << "SendPingConfig: range" << m_range << "gain" << m_gain_percent;
            m_cancel_timeout = boost::make_shared<bool>(false);
            m_async_service->callAfterTime(
                boost::bind(&GeminiSonar::prepareNextPing, this, "timeout!", m_cancel_timeout),
                boost::posix_time::milliseconds(200 + 2 * 1000*(2.0*m_range/m_sos)) // twice as long, plus 200ms as a ping should take
            );
            GEM_SendGeminiPingConfig();
        }

    // Message Observer:
        virtual void onGeminiControlMessage(GeminiControlMessage_ptr m){
            debug() << "received GeminiControlMessage:" << *m;
            m_range = m->range();
            m_gain_percent = m->gain();
            m_range_lines = m->rangeLines();
            m_inter_ping_musec = m->interPingPeriod() * 1e6 + 0.5;
            m_ping_continuous = m->continuous();
            
            if(!initialised()){
                debug() << "config received before init: will not ping yet!";
                return;
            }
            if(m_currently_pinging){
                debug() << "already pinging!";
                return;
            }

            applyConfigAndPing();
        }

    private:
        static std::string fmtIp(uint32_t ip4_addr){
            uint8_t ipchrs[4] = {
                static_cast<uint8_t>((ip4_addr >> 24) & 0xff),
                static_cast<uint8_t>((ip4_addr >> 16) & 0xff),
                static_cast<uint8_t>((ip4_addr >>  8) & 0xff),
                static_cast<uint8_t>((ip4_addr >>  0) & 0xff)
            };
            return mkStr() << int(ipchrs[0]) << "."
                           << int(ipchrs[1]) << "."
                           << int(ipchrs[2]) << "."
                           << int(ipchrs[3]);
        }
        static std::string fmtLinkType(uint32_t link_type){
            mkStr r;
            if(link_type & 0x1) r << "VDSL ";
            if(link_type & 0x2) r << "10Mbit ";
            if(link_type & 0x4) r << "100Mbit ";
            if(link_type & 0x8) r << "1GBit ";
            if(link_type & 0xfffffff0)
                r << std::hex << link_type;
            return r;
        }
        static std::string fmtShutdownStatus(uint32_t status){
            mkStr r;
            if(status & 0x1) r << "Over-Temperature! ";
            if(status & 0x2) r << "Out of water! ";
            return r;
        }

        void onStatusPacket(CGemStatusPacket const* status_packet){
            // state transitions to manage the connection:
            // uninitialised
            {
            debug(8) << "onStatusPacket:"
                    << "m_firmwareVer =" << status_packet->m_firmwareVer
                    << "m_sonarId =" << status_packet->m_sonarId
                    << "m_sonarFixIp =" << status_packet->m_sonarFixIp
                    << "m_sonarAltIp =" << status_packet->m_sonarAltIp
                    << "m_surfaceIp =" << status_packet->m_surfaceIp
                    << "m_flags =" << status_packet->m_flags
                    << "m_vccInt =" << status_packet->m_vccInt
                    << "m_vccAux =" << status_packet->m_vccAux
                    << "m_dcVolt =" << status_packet->m_dcVolt
                    << "m_dieTemp =" << status_packet->m_dieTemp
                    << "m_tempX =" << status_packet->m_tempX
                    << "m_vga1aTemp =" << fmtTemp(status_packet->m_vga1aTemp)
                    << "m_vga1bTemp =" << fmtTemp(status_packet->m_vga1bTemp)
                    << "m_vga2aTemp =" << fmtTemp(status_packet->m_vga2aTemp)
                    << "m_vga2bTemp =" << fmtTemp(status_packet->m_vga2bTemp)
                    << "m_psu1Temp =" << fmtTemp(status_packet->m_psu1Temp)
                    << "m_psu2Temp =" << fmtTemp(status_packet->m_psu2Temp)
                    << "m_currentTimestampL =" << status_packet->m_currentTimestampL
                    << "m_currentTimestampH =" << status_packet->m_currentTimestampH
                    << "m_transducerFrequency =" << status_packet->m_transducerFrequency
                    << "m_subnetMask =" << status_packet->m_subnetMask
                    << "m_TX1Temp =" << fmtTemp(status_packet->m_TX1Temp)
                    << "m_TX2Temp =" << fmtTemp(status_packet->m_TX2Temp)
                    << "m_TX3Temp =" << fmtTemp(status_packet->m_TX3Temp)
                    << "m_BOOTSTSRegister =" << status_packet->m_BOOTSTSRegister
                    << "m_shutdownStatus =" << status_packet->m_shutdownStatus
                    << "m_dieOverTemp =" << status_packet->m_dieOverTemp
                    << "m_vga1aShutdownTemp =" << fmtTemp(status_packet->m_vga1aShutdownTemp)
                    << "m_vga1bShutdownTemp =" << fmtTemp(status_packet->m_vga1bShutdownTemp)
                    << "m_vga2aShutdownTemp =" << fmtTemp(status_packet->m_vga2aShutdownTemp)
                    << "m_vga2bShutdownTemp =" << fmtTemp(status_packet->m_vga2bShutdownTemp)
                    << "m_psu1ShutdownTemp =" << fmtTemp(status_packet->m_psu1ShutdownTemp)
                    << "m_psu2ShutdownTemp =" << fmtTemp(status_packet->m_psu2ShutdownTemp)
                    << "m_TX1ShutdownTemp =" << fmtTemp(status_packet->m_TX1ShutdownTemp)
                    << "m_TX2ShutdownTemp =" << fmtTemp(status_packet->m_TX2ShutdownTemp)
                    << "m_TX3ShutdownTemp =" << fmtTemp(status_packet->m_TX3ShutdownTemp)
                    << "m_linkType =" << status_packet->m_linkType
                    << "m_VDSLDownstreamSpeed1 =" << status_packet->m_VDSLDownstreamSpeed1
                    << "m_VDSLDownstreamSpeed2 =" << status_packet->m_VDSLDownstreamSpeed2
                    << "m_macAddress1 =" << status_packet->m_macAddress1
                    << "m_macAddress2 =" << status_packet->m_macAddress2
                    << "m_macAddress3 =" << status_packet->m_macAddress3
                    << "m_VDSLUpstreamSpeed1 =" << status_packet->m_VDSLUpstreamSpeed1
                    << "m_VDSLUpstreamSpeed2 =" << status_packet->m_VDSLUpstreamSpeed2;
            info() << "sonarId:"   << status_packet->m_sonarId
                    << "FixIp:"     << fmtIp(status_packet->m_sonarFixIp)
                    << "AltIp:"     << fmtIp(status_packet->m_sonarAltIp)
                    << "SurfaceIp:" << fmtIp(status_packet->m_surfaceIp)
                    << "NetMask:"   << fmtIp(status_packet->m_subnetMask)
                    << "ShutdownStatus:" << fmtShutdownStatus(status_packet->m_shutdownStatus)
                    << "LinkType:"  << fmtLinkType(status_packet->m_linkType);
            }
            if(m_conn_state.sonarId == 0){
                if(status_packet){
                    m_conn_state.sonarId = status_packet->m_sonarId;
                    m_conn_state.sonarAltIp = status_packet->m_sonarAltIp;
                }
                if(m_conn_state.sonarId != 0){
                    lock_t l(m_gem_mux);
                    GEM_SetDLLSonarID(m_conn_state.sonarId);
                    unsigned char ip_chrs[4] = {
                        static_cast<uint8_t>((m_conn_state.sonarAltIp >> 24) & 0xff),
                        static_cast<uint8_t>((m_conn_state.sonarAltIp >> 16) & 0xff),
                        static_cast<uint8_t>((m_conn_state.sonarAltIp >> 8) & 0xff),
                        static_cast<uint8_t>((m_conn_state.sonarAltIp >> 0) & 0xff)
                    };
                    unsigned char nm_chrs[4] = {
                        255,255,255,0
                    };
                    // note that the UseAltSonarIPAddress has to be called with
                    // the sonar's existing alternate IP address: this call
                    // cannot be used to set the alternate IP address (there is
                    // an API call for that, but best to do it through
                    // Tritech's own software, since it should be a
                    // set-and-forget thing)
                    // this prints in the right format (according to the
                    // manual), so if endian issues happen, they'll be visible
                    debug() << "GEM_UseAltSonarIPAddress("
                            << (int)ip_chrs[0] << "."
                            << (int)ip_chrs[1] << "."
                            << (int)ip_chrs[2] << "."
                            << (int)ip_chrs[3] << "/"
                            << (int)nm_chrs[0] << "."
                            << (int)nm_chrs[1] << "."
                            << (int)nm_chrs[2] << "."
                            << (int)nm_chrs[3] << ")";
                    // there also exists SetAltSonarIpAddress, which programs
                    // it in flash and requires a reboot
                    GEM_UseAltSonarIPAddress(
                        ip_chrs[0], ip_chrs[1], ip_chrs[2], ip_chrs[3],
                        nm_chrs[0], nm_chrs[1], nm_chrs[2], nm_chrs[3]
                    );
                    // otherwise, the default address of 10.61.19.200 is used
                    // to talk to the head
                    GEM_TxToAltIPAddress(true);
                    // 0 = only ping on receipt of ping configuration message,
                    // 1 = ping continuously
                    GEM_SetPingMode(0);
                    GEM_SetInterPingPeriod(m_inter_ping_musec);
                    //
                    GEM_SetVelocimeterMode(
                        0, // 0 = auto gain
                           // 1 = manual gain (actually, auto is always true anyway)
                        0  // 0 = use velocimeter calculated speed of sound,
                           // 1 = use speed set in config message
                    );
                    // pinging out of water is very useful for testing
                    GEM_SetExtModeOutOfWaterOverride(1);
                    m_conn_state.initialised = true;
                }
            }
        }

        static void CallBackFn(int eType, int /*len*/, char *data){
            CGemPingHead const* ping_head = nullptr;
            CGemPingLine const* ping_data = nullptr;
            CGemPingTail const* ping_tail = nullptr;
            CGemPingTailExtended const* ping_tail_ex = nullptr;
            CGemStatusPacket const* status_packet = nullptr;
            CGemAcknowledge const* acknowledge = nullptr;
            CGemBearingData const* bearing_data = nullptr;
            uint8_t const* unknown_data = nullptr;

            debug(8) << "RX:" << eType;
            switch(eType){
                case PING_HEAD:
                    debug(6) << "RX: ping head";
                    ping_head = (CGemPingHead*)data;
                    break;

                case PING_DATA:
                    //debug(7) << "RX: ping line: len=" << len;
                    ping_data = (CGemPingLine*)data;
                    break;

                case PING_TAIL:
                    // Normally we get PING_TAIL_EX, but this happens if the
                    // retry data structures were never created by the library,
                    // so there's no extra data for the library to add.
                    // (not sure why this might be, but it does happen)
                    ping_tail = (CGemPingTail*)data;
                    debug(2) << BashColour::Purple
                            << "RX: ping tail: id=" << int(ping_tail->m_pingID);
                    break;

                case PING_TAIL_EX:
                    ping_tail_ex = (CGemPingTailExtended*)data;
                    debug(6) << "RX: ping tail ex: id=" << int(ping_tail_ex->m_pingID);
                    debug(2) << BashColour::Purple
                            << "retry 1:"       << ping_tail_ex->m_firstPassRetries
                            << "retry 2:"       << ping_tail_ex->m_secondPassRetries
                            << "tail retries:"  << ping_tail_ex->m_tailRetries
                            << "RX Error count:"<< ping_tail_ex->m_recvErrorCount
                            << "RX Total:"      << ping_tail_ex->m_packetCount
                            << "Lines lost:"    << ping_tail_ex->m_linesLostThisPing;
                    break;

                case GEM_STATUS:
                    debug(6) << "RX: status";
                    status_packet = (CGemStatusPacket*)data;
                    break;

                case GEM_ACKNOWLEDGE:
                    debug(6) << "RX: ack";
                    acknowledge = (CGemAcknowledge*)data;
                    break;

                case GEM_BEARING_DATA:
                    debug(6) << "RX: bearing data";
                    bearing_data = (CGemBearingData*)data;
                    break;

                case GEM_UNKNOWN_DATA:
                    debug() << "GEM: unknown data";
                    unknown_data = (uint8_t const*)data;
                    break;

                case GEM_IP_CHANGED:
                    debug() << "GEM: ip changed";
                    break;

                default:
                    debug() << "GEM: unknown type:" << eType;
                    break;
            }

            // actually dispatch messages to anyone interested:
            if(the_sonar){
                // first use status packet to drive state transitions (eg, for
                // initialisation):
                if(status_packet)
                    the_sonar->onStatusPacket(status_packet);

                if(ping_tail || ping_tail_ex)
                    the_sonar->prepareNextPing("ping tail", the_sonar->m_cancel_timeout);

                boost::lock_guard<boost::recursive_mutex> l(the_sonar->m_observers_lock);
                for (observer_ptr_t& p : the_sonar->m_observers){
                    if(ping_head)
                        p->onCGemPingHead(ping_head, the_sonar->range());
                    else if(ping_data)
                        p->onCGemPingLine(ping_data);
                    else if(ping_tail)
                        p->onCGemPingTail(ping_tail, the_sonar->m_range_lines);
                    else if(ping_tail_ex)
                        p->onCGemPingTailExtended(ping_tail_ex, the_sonar->m_range_lines);
                    else if(status_packet)
                        p->onCGemStatusPacket(status_packet);
                    else if(acknowledge)
                        p->onCGemAcknowledge(acknowledge);
                    else if(bearing_data)
                        p->onCGemBearingData(bearing_data);
                    else if(unknown_data)
                        p->onUnknownData(unknown_data);
                }
            }else{
                error() << "No GeminiSonar object to receive callback";
            }
        }

        // precondition: m_gem_mux is locked from a previous ping: this unlocks
        // it unconditionally
        //
        // Another ping will only be prepared if m_ping_continuous is true
        void prepareNextPing(const std::string& descr, boost::shared_ptr<bool> cancel_timeout){
            debug(3) << "prepareNextPing" << descr;

            lock_t l(m_cancel_timeout_mux);
            if(*cancel_timeout)
                m_currently_pinging = false;
                return;
            *cancel_timeout = true;

            if(m_ping_continuous){
                // set off another ping after m_inter_ping_musec:
                m_async_service->callAfterMicroseconds(
                    boost::bind(&GeminiSonar::applyConfigAndPing, the_sonar),
                    m_inter_ping_musec
                );
            }
            else {
                m_currently_pinging = false;
            }
            m_gem_mux.unlock();
        }

        float m_sos;
        uint16_t m_sonar_id;
        uint32_t m_inter_ping_musec;
        float m_range;
        float m_gain_percent;
        uint32_t m_range_lines;
        bool m_ping_continuous;

        volatile struct{
            bool ok;
            bool initialised;
            uint16_t sonarId;
            uint32_t sonarAltIp;
        } m_conn_state;
        
        bool m_currently_pinging;
        
        boost::shared_ptr<AsyncService> m_async_service;

        mutex_t m_cancel_timeout_mux;
        boost::shared_ptr<bool> m_cancel_timeout;

        mutex_t m_gem_mux;

        // there should only be one instance of this at a time, keep track of
        // it so that callbacks can be dispatched
        static GeminiSonar* the_sonar;
};
GeminiSonar* GeminiSonar::the_sonar = nullptr;

} // namespace cauv

using namespace cauv;

static GeminiNode* node;


GeminiNode::GeminiNode()
    : CauvNode("Gemini"){
}

void GeminiNode::onRun()
{

    if(!m_sonar){
        error() << "no sonar device";
        return;
    }

    /*
    joinGroup("sonarctl");
    joinGroup("telemetry");

    boost::shared_ptr<SpreadSonarObserver> spreadSonarObserver = boost::make_shared<SpreadSonarObserver>(mailbox());
    m_sonar->addObserver(spreadSonarObserver);
    addMessageObserver(spreadSonarObserver);

#ifdef DISPLAY_SONAR
    m_sonar->addObserver(boost::make_shared<DisplaySonarObserver>(m_sonar));
#endif

    addMessageObserver(boost::make_shared<SonarControlMessageObserver>(m_sonar));
    */

    m_sonar->init();

    while(!m_sonar->initialised()){
        debug() << "waiting for init...";
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    subMessage(GeminiControlMessage());
    m_sonar->addObserver(boost::make_shared<ReBroadcaster>(boost::ref(*this)));
    this->addMessageObserver(m_sonar);

    m_sonar->autoConfig(6.0f, 40);

    while (true) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        GEM_SendGeminiStayAlive();
    }
}



void GeminiNode::addOptions(po::options_description& desc,
                            po::positional_options_description& pos)
{
    CauvNode::addOptions(desc, pos);

    desc.add_options()
        ("sonar_id,d", po::value<uint16_t>()->required(), "The sonar ID (eg 1)")
    ;

    pos.add("sonar_id", 1);

}

int GeminiNode::useOptionsMap(po::variables_map& vm,
                              po::options_description& desc)
{
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;

    uint16_t sonar_id = vm["sonar_id"].as<uint16_t>();
    m_sonar = boost::make_shared<GeminiSonar>(sonar_id);

    /*
    if(!m_sonar->ok()){
        error() << "could not open device" << device;
        return 2;
    }
    */

    return 0;
}

void cleanup()
{
    info() << "Cleaning up...";
    CauvNode* oldnode = node;
    node = nullptr;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig)
{
    std::cout << std::endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    node = new GeminiNode();
    try{
        if(node->parseOptions(argc, argv) == 0)
            node->run(false);
    }catch(boost::program_options::error& e){
        error() << e.what();
    }
    cleanup();
    return 0;
}

