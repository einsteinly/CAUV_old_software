/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include <string>
#include <vector>
#include <stdexcept>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/ref.hpp>

#include <Eigen/Dense>

#include <common/cauv_node.h>
#include <common/msg_classes/wgs84_coord.h>
#include <common/msg_classes/north_east_depth.h>

#include <generated/message_observers.h>
#include <generated/types/GeminiControlMessage.h>
#include <generated/types/SimPositionMessage.h>
#include <generated/types/SonarImageMessage.h>

#include <utility/coordinates.h>
#include <utility/time.h>
#include <utility/rounding.h>
#include <utility/math.h>

#include <debug/cauv_debug.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// !!!!!!! Note that this is a temporary measure: sonar simulation should be
//         done in the main sim using the depth buffer and other magic, which
//         would be a million times more efficient than this.

using namespace cauv;
namespace po = boost::program_options;

// Image origin is BOTTOM LEFT, rows (y) increase upwards (North), columns (x) increase
// to the right (East)
class Environment{
    public:
        Environment(
            std::string const& env_file,
            float origin_x_px,
            float origin_y_px,
            float metres_per_px,
            WGS84Coord const& datum
        ) : m_image(),
            m_datum(datum),
            m_px_origin(origin_x_px, origin_y_px),
            m_metres_per_px(metres_per_px)
        { 
            m_image = cv::imread(env_file.c_str()); 
            if(m_image.channels() == 3){
                cv::Mat tmp;
                cv::cvtColor(m_image, tmp, CV_RGB2GRAY);
                m_image = tmp;
            }
            if(m_image.channels() != 1 || m_image.depth() != CV_8U)
                throw std::runtime_error("use a single-channel 8-bit image!");
            debug() << "environment size is"
                    << m_image.rows * m_metres_per_px << "x"
                    << m_image.cols * m_metres_per_px;
        }
    
    Eigen::Vector2f xyToImgCoords(Eigen::Vector2f const& xy) const{
        return (xy / m_metres_per_px) - m_px_origin;
    }

    Eigen::Vector2f latLongToImgCoords(WGS84Coord const& ll) const{
        const NorthEastDepth ned = ll - m_datum;
        return xyToImgCoords(Eigen::Vector2f(ned.east(), ned.north()));
    }

    std::vector<uint8_t> getLine(
        WGS84Coord const& ll, float bearing_start, float bearing_end, float length, int res
    ) const{
        // bearing start/end from north of line centre, degrees
        // length in real world metres
        // res in number-of-pixels
        const Eigen::Vector2f image_xy = latLongToImgCoords(ll);
        const float angle_end = radians(90-bearing_start);
        const float angle_start = radians(90-bearing_end);
        return getLineImageCoords(
            image_xy, angle_start, angle_end, length / m_metres_per_px, res
        );
    }

    std::vector<uint8_t> getLineImageCoords(
        Eigen::Vector2f const& xy, float start_angle, float end_angle, float length, int res
    ) const{
        // xy:    in pixel-units from pixel origin
        // angle start/end:  measured in radians ccw from x-axis (note that a line at 90
        //         degrees points *negative* y in this system)
        // length: in px
        std::vector<uint8_t> r(res, 0);

        assert(start_angle < end_angle);
        start_angle = wrapAngleIntoPlusOrMinusPi(start_angle);
        end_angle = wrapAngleIntoPlusOrMinusPi(end_angle);
        
        const Eigen::Vector2f start_direction(std::cos(start_angle), std::sin(start_angle));
        const Eigen::Vector2f end_direction(std::cos(end_angle), std::sin(end_angle));
        
        const int image_sizex = m_image.cols;
        const int image_sizey = m_image.rows;
        
        Eigen::Vector2f c1 = xy;
        Eigen::Vector2f c2 = xy;
        Eigen::Vector2f c3 = xy;
        Eigen::Vector2f c4 = xy;
        float rmin = 0;
        float rmin_sq = 0;
        float rmax = 0;
        float rmax_sq = 0;
        for(int bin = 0; bin < res; bin++){
            float support_px = 0.0;
            float value = 0.0;
            //   
            //     - _ _
            //    |       - _ 
            //    |           - 
            // c4 |' - -        /
            //    |      ' .  /
            //    |__       / c2
            // c3 |  ''.. /
            //    |     / c1
            //    |   /
            //    | /
            //    o
            //
            //    y
            //    |
            //    o--> x
            //
            rmin = rmax;
            rmax = (bin+1) * float(length)/res;
            rmin_sq = rmax_sq;
            rmax_sq = rmax * rmax;
            c1 = c2;
            c2 = xy + start_direction * rmax;
            c3 = c4;
            c4 = xy + end_direction * rmax;
            const Rect bbox = boundingRect(c1, c2, c3, c4);
            
            for(int x = bbox.min[0]; x < bbox.max[0]+1; x++){
                const float xlocal = x - xy[0];
                for(int y = bbox.min[1]; y < bbox.max[1]+1; y++){ 
                    if(x >= 0 && y > 0 && x < image_sizex && y <= image_sizey){
                        const float ylocal = y - xy[1];
                        const float r_sq = xlocal*xlocal + ylocal*ylocal;
                        if(r_sq >= rmin_sq && r_sq <= rmax_sq){
                            const float xy_angle = std::atan2(ylocal, xlocal);
                            if(angleIsInRange(start_angle, xy_angle, end_angle)){
                                support_px += 1;
                                // convert y into Image coordinates: origin top
                                value += m_image.at<uint8_t>(image_sizey - y,x);
                            }
                        }
                    }
                }
            }

            if(support_px > 0){
                r[bin] = uint8_t(0.5 + value / support_px);
            }else{
                // no support? just sample the closest pixel:
                Eigen::Vector2f xy_mid = (c1 + c2 + c3 + c4) / 4;
                const int x = int(0.5 + xy_mid[0]);
                const int y = int(0.5 + xy_mid[1]);
                if(x >= 0 && y > 0 && x < image_sizex && y <= image_sizey){
                    // convert y into Image coordinates: origin top
                    r[bin] = m_image.at<uint8_t>(image_sizey - y,x);
                }
            }
        }
        return r;
    }

    private:
        static float wrapAngleIntoPlusOrMinusPi(float angle){
            while(angle > M_PI)
                angle -= M_PI*2;
            while(angle < -M_PI)
                angle += M_PI*2;
            return angle;
        }
        
        struct Rect{
            Eigen::Vector2f min;
            Eigen::Vector2f max;
        };
        static Rect boundingRect(
            Eigen::Vector2f const& a,
            Eigen::Vector2f const& b,
            Eigen::Vector2f const& c,
            Eigen::Vector2f const& d
        ){
            Rect r = {a, a};
            Eigen::Vector2f min = a;
            Eigen::Vector2f max = a;
            for(int i = 0; i < 2; i++){
                if(b[i] < r.min[i]) r.min[i] = b[i];
                if(b[i] > r.max[i]) r.max[i] = b[i];
                if(c[i] < r.min[i]) r.min[i] = c[i];
                if(c[i] > r.max[i]) r.max[i] = c[i];
                if(d[i] < r.min[i]) r.min[i] = d[i];
                if(d[i] > r.max[i]) r.max[i] = d[i];
            }
            return r;
        }

        static bool angleIsInRange(float start_angle, float angle, float end_angle){
            if(start_angle <= end_angle)
                return angle >= start_angle && angle <= end_angle;
            else
                return angle <= start_angle || angle >= end_angle;
        }

        cv::Mat m_image;
        WGS84Coord m_datum;
        Eigen::Vector2f m_px_origin;
        float m_metres_per_px;
};

class FakeGemini: public MessageObserver{
    public:
        FakeGemini(
            Environment const& env,
            CauvNode& node
        ) : MessageObserver(),
            m_environment(env),
            m_node(node),
            m_inited(0),
            m_position(),
            m_gemcontrol(){
            
        }

        void start(){
            mainLoop();
        }

        virtual void onSimPositionMessage(SimPositionMessage_ptr p){
            m_inited |= GotSimPos;
            m_position = p;
        }

        virtual void onGeminiControlMessage(GeminiControlMessage_ptr p){
            m_inited |= GotGemControl;
            m_gemcontrol = p;
        }

        void mainLoop() const{
            for(;;){
                if(!(m_inited & GotGemControl)){
                    debug() << "no gemini control messages received";
                    msleep(1000);
                    continue;
                }
                const GeminiControlMessage_ptr gemcontrol = m_gemcontrol; 
                msleep(gemcontrol->interPingPeriod() * 1000);

                if(!(m_inited & GotSimPos)){
                    debug() << "no sim position messages received";
                    continue;
                }
                SimPositionMessage_ptr p = m_position;
                const WGS84Coord pos = p->location(); 
                const float yaw = p->orientation().yaw;
                const TimeStamp ts = now();
                
                const static std::vector<float> bearing_bins = bearingBinDegrees();
                const static std::vector<int> bearing_bins_otw = bearingBinsOTW();
                
                const float range = gemcontrol->range();
                const float rangeLines = gemcontrol->rangeLines();
                const int num_beams = 256;

                debug() << "at:" << pos
                        << " =pixels:" << m_environment.latLongToImgCoords(pos).transpose()
                        << " look bearing:" << yaw;

                std::vector<uint8_t> beams;
                beams.reserve(num_beams*rangeLines);

                for(std::size_t i = 0; i+1 < bearing_bins.size(); i++){
                    const float beam_start = bearing_bins[i] + yaw;
                    const float beam_end = bearing_bins[i+1] + yaw;
                    assert(beam_end > beam_start);
                    const std::vector<uint8_t> beam = m_environment.getLine(
                        pos, beam_start, beam_end, range, rangeLines
                    );
                    beams.insert(beams.end(), beam.begin(), beam.end());
                }

                // oops, need to swap order...
                std::vector<uint8_t> beams_swapped;
                beams_swapped.reserve(beams.size());
                for(int i = 0; i < rangeLines; i++)
                    for(int b = 0; b < num_beams; b++)
                        beams_swapped.push_back(beams[b * rangeLines + i]);
                
                m_node.send(boost::make_shared<SonarImageMessage>(
                    SonarID::Gemini, PolarImage(
                        beams_swapped,
                        ImageEncodingType::RAW_uint8_1,
                        bearing_bins_otw,
                        0.0,
                        range,
                        range / rangeLines,
                        ts
                    )
                ));
            }
        }

    private:
        static std::vector<float>  bearingBinDegrees(){
            std::vector<float> r(257);
            for(uint32_t beam = 0; beam <= 256; beam++){
                // see Gemini Interface Specification Appendix B
                double radians = std::asin(((2*(beam+0.5) - 256) / 256) * 0.86602540);
                r[beam] = radians * (180.0 / M_PI);
            }
            return r;
        }
        static std::vector<int32_t> bearingBinsOTW(uint32_t num_beams=256){
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


        Environment m_environment;
        CauvNode& m_node;

        enum {GotSimPos = 1, GotGemControl = 2};
        int m_inited;

        SimPositionMessage_ptr   m_position;
        GeminiControlMessage_ptr m_gemcontrol;
};


class FakeGeminiNode: public CauvNode{
    public:
        FakeGeminiNode() : CauvNode("FakeGem"){ }
    protected:
        virtual void onRun(){
            if(!m_sim){
                error() << "not initialised";
                return;
            }

            // required message subscriptions:
            subMessage(GeminiControlMessage());
            subMessage(SimPositionMessage());

            addMessageObserver(m_sim);
            m_sim->start();
        }
        virtual void addOptions(po::options_description& desc,
                                po::positional_options_description& pos){
            CauvNode::addOptions(desc, pos);

            desc.add_options()
                ("environment,e", po::value<std::string>()->required(), "The environment file (single-channel image)")
                ("scale,s", po::value<float>()->required(), "The environment scale (metres per pixel)")
                ("origin-x,x", po::value<float>(), "x-origin of environment (px from top-left)")
                ("origin-y,y", po::value<float>(), "y-origin of environment (px from top-left)")
            ;

        }
        virtual int useOptionsMap(po::variables_map& vm,
                                  po::options_description& desc){
            int ret = CauvNode::useOptionsMap(vm, desc);
            if(ret) return ret;
            
            const std::string env_file = vm["environment"].as<std::string>();
            const float scale = vm["scale"].as<float>();
            float origin_x_px = 0;
            float origin_y_px = 0;
            if(vm.count("origin-x")) origin_x_px = vm["origin-x"].as<float>();
            if(vm.count("origin-y")) origin_y_px = vm["origin-y"].as<float>();

            m_sim = boost::make_shared<FakeGemini>(Environment(
                env_file, origin_x_px, origin_y_px, scale, WGS84Coord::fromDatum("river cam")
            ), boost::ref(*this));

            return 0;
        }
    private:
        boost::shared_ptr<FakeGemini> m_sim;
};


static FakeGeminiNode* node;

void cleanup(){
    info() << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig){
    std::cout << std::endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv){
    signal(SIGINT, interrupt);
    node = new FakeGeminiNode();
    try{
        if(node->parseOptions(argc, argv) == 0)
            node->run(false);
    }catch(boost::program_options::error& e){
        error() << e.what();
    }
    cleanup();
    return 0;
}
