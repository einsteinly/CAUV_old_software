#include <iostream>
#include <sstream>
#include <stdint.h>
#include <curses.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <common/cauv_node.h>
#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

using namespace std;

class PervertNode : public CauvNode
{
    public: 
        PervertNode() : CauvNode("Pervert")
        {
            joinGroup("control");
    
            initscr();             /* start the curses mode */
        }
        ~PervertNode()
        {
            endwin();
        }

    protected:
        virtual void onRun()
        {
            CauvNode::onRun();
            
            //int rows, cols;
            //getmaxyx(stdscr, rows, cols);      /* get the number of rows and columns */
            int i = 0;
            while(true)
            {
                i = 1 - i;
                mvprintw(0,4,"Image");
                mvprintw(0,16, i?"#":0);
                refresh();
                msleep(10);
            }
        }
};

static PervertNode* node;

void cleanup()
{
    info() << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    
    node = new PervertNode();

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;
    
    node->run();
    
    cleanup();
    return 0;
}
