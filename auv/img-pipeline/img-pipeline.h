/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __IMAGE_PIPELINE_H__
#define __IMAGE_PIPELINE_H__

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

#include <generated/message_observers.h>


namespace cauv{
namespace imgproc{

class Scheduler;
class ImageProcessor;

class ImagePipelineNode : public CauvNode,
                          public MessageObserver
{
    public:
        ImagePipelineNode();

        void spawnNewPipeline(std::string const& with_name);
        
        // message observer: new pipelines are created based on these messages
        // only:
        virtual void onAddNodeMessage(AddNodeMessage_ptr m);
        virtual void onRemoveNodeMessage(RemoveNodeMessage_ptr m);
        virtual void onGraphRequestMessage(GraphRequestMessage_ptr m);
        virtual void onClearPipelineMessage(ClearPipelineMessage_ptr m);
        virtual void onSetPipelineMessage(SetPipelineMessage_ptr m);
    
    protected:
        virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);

    private:
        std::string m_pipeline_name_root;
        std::map<std::string,  boost::shared_ptr<ImageProcessor> > m_pipelines;
        boost::shared_ptr<Scheduler> m_scheduler;        
};

} // namespace imgproc
} // namespace cauv

#endif //__IMAGE_PIPELINE_H__

