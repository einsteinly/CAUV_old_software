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

#include <QtGui>
#include <QTimer>

#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/program_options.hpp>

#include <gui/core/model/node.h>

#include <common/cauv_node.h>

#include "styles/style.h"
#include "gui/plugins/fluidity/view.h"

class ScratchNode: public cauv::CauvNode, public boost::enable_shared_from_this<ScratchNode>{
    public:
        ScratchNode(std::string const& n, int argc, char* argv[])
            : cauv::CauvNode(n), m_argc(argc), m_argv(argv), m_exit_status(0),
              m_pipeline_name(""){
        }

        void addOptions(boost::program_options::options_description& desc,
                        boost::program_options::positional_options_description& pos){
            namespace po = boost::program_options;
            CauvNode::addOptions(desc, pos);
            
            desc.add_options()
                ("pipeline,n",
                po::value<std::string>()->default_value("default"), "pipeline to connect to");
        }
        int useOptionsMap(boost::program_options::variables_map& vm,
                          boost::program_options::options_description& desc){
            namespace po = boost::program_options;
            int ret = CauvNode::useOptionsMap(vm, desc);
            if (ret != 0) return ret;

            m_pipeline_name = vm["pipeline"].as<std::string>();

            return 0;
        }

        void onRun(){
            QApplication app(m_argc, m_argv);

            QIcon icon;
            icon.addFile(QString::fromUtf8(":/resources/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
            app.setWindowIcon(icon);

            QApplication::setStyle(new cauv::gui::CauvStyle());

            boost::shared_ptr<cauv::gui::Node> n = boost::make_shared<cauv::gui::Node>("scratch", cauv::gui::nodeType<cauv::gui::Node>());
            cauv::gui::f::FView* view = new cauv::gui::f::FView(shared_from_this(), m_pipeline_name, n);
            view->show();

            m_exit_status = app.exec();

            stopNode();
        }

        int m_argc;
        char** m_argv;
        int m_exit_status;        
        std::string m_pipeline_name;

};


int main(int argc, char *argv[]){
    boost::shared_ptr<ScratchNode> node = boost::make_shared<ScratchNode>("scratch", argc, argv);
    if(node->parseOptions(argc, argv))
        return 0;

    node->run(false);

    return node->m_exit_status;
}
