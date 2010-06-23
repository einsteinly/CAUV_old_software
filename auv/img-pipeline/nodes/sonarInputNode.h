#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstring>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"


class SonarInputNode: public InputNode{
    public:
        SonarInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : InputNode(sched, pl, t), m_data(){
            // InputNode stuff: subscribe to sonar data
            m_subscriptions.insert(SonarData);

            // registerInputID()
            
            // one output:
            registerOutputID<image_ptr_t>("image");
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug() << "SonarInputNode::doWork";
            
            boost::shared_ptr<SonarDataMessage const> m = latestSonarDataMessage();

            if(m_data.size() &&
               (m_data.begin()->range != m->line().range) ||
               (m_data.begin()->data.size() != m->line().data.size())){
                m_data.clear();
            }
            
            data_set_t::iterator i = m_data.find(m->line());
            if(i != m_data.end())
                m_data.erase(i);
            m_data.insert(m->line());
            
            r["image"] = combineAccumulatedData();
            
            // setAllowQueue() is called by InputNode when a new SonarDataMessage
            // is available
            clearAllowQueue();

            return r;
        }

    private:
        cv::Mat cvMatRowFromLine(std::vector< uint8_t > const& l){
            cv::Mat r(1, l.size(), CV_8UC1);
            std::memcpy(r.data, l.data(), l.size());
            return r;
        }

        image_ptr_t combineAccumulatedData(){
            image_ptr_t r = boost::make_shared<Image>(Image::src_sonar);        
            if(!m_data.size())
                return r;
            int columns = m_data.begin()->data.size();
            int rows = m_data.size();

            cv::Mat rm(rows, columns, CV_8UC1, 0);
            data_set_t::iterator it = m_data.begin();
            for(int i = 0; it != m_data.end() && i < rows; i++, it++)
                rm.row(i) = cvMatRowFromLine(it->data);
            r->cvMat() = rm;
            return r;
        }
        
        struct CompareDataLinesByBearing{
            bool operator()(SonarDataLine const& l, SonarDataLine const& r) const{
                return l.bearing < r.bearing;
            }
        };
        typedef std::set<SonarDataLine, CompareDataLinesByBearing> data_set_t;
        data_set_t m_data;
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __SONAR_INPUT_NODE_H__

